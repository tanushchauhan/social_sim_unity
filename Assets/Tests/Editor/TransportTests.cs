using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using NUnit.Framework;
using UnityEngine;

public class TransportTests
{
    private static Type Connection => AppDomain.CurrentDomain.GetAssemblies()
        .Select(a => a.GetType("Unity.Robotics.ROSTCPConnector.ROSConnection")).First(t => t != null);
    private static readonly BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
    private static async Task Exact(Stream stream, byte[] buffer)
    {
        int position = 0;
        while (position < buffer.Length)
        {
            int read = await stream.ReadAsync(buffer, position, buffer.Length-position);
            if (read == 0) throw new EndOfStreamException();
            position += read;
        }
    }
    private static async Task<string> Frame(Stream stream)
    {
        var bytes = new byte[4];
        await Exact(stream, bytes);
        byte[] topic = new byte[BitConverter.ToInt32(bytes, 0)];
        await Exact(stream, topic);
        await Exact(stream, bytes);
        await Exact(stream, new byte[BitConverter.ToInt32(bytes, 0)]);
        return Encoding.ASCII.GetString(topic).TrimEnd('\0');
    }
    private static byte[] Wire(string topic, byte[] payload)
    {
        byte[] name = Encoding.ASCII.GetBytes(topic);
        return BitConverter.GetBytes(name.Length).Concat(name).Concat(BitConverter.GetBytes(payload.Length))
            .Concat(payload).ToArray();
    }
    [Test] public void DisconnectJoinsTheWorkerAndDrainsEndpointDataBeforeClosing()
    {
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        var go = new GameObject("transport ownership");
        try
        {
            Task server = Task.Run(async () => {
                using (TcpClient client = await listener.AcceptTcpClientAsync())
                using (NetworkStream stream = client.GetStream())
                {
                    byte[] handshake = Wire("__handshake", Encoding.UTF8.GetBytes("{}"));
                    await stream.WriteAsync(handshake, 0, handshake.Length);
                    while (await Frame(stream) != "__disconnect") { }
                    // An application must drain this final response before closing.
                    byte[] final = Wire("/final", new byte[128*1024]);
                    await stream.WriteAsync(final, 0, final.Length);
                    client.Client.Shutdown(SocketShutdown.Send);
                    Assert.That(await stream.ReadAsync(new byte[1], 0, 1), Is.Zero);
                }
            });
            Component connection = go.AddComponent(Connection);
            Connection.GetProperty("RosPort").SetValue(connection, ((IPEndPoint)listener.LocalEndpoint).Port);
            Connection.GetMethod("Connect", Type.EmptyTypes).Invoke(connection, null);
            Task worker = (Task)Connection.GetField("m_ConnectionTask", Private).GetValue(connection);
            Assert.That(SpinWait.SpinUntil(() => Connection.GetField("m_IncomingMessages", Private)
                .GetValue(connection).GetType().GetProperty("Count").GetValue(
                    Connection.GetField("m_IncomingMessages", Private).GetValue(connection)).Equals(1), 3000), Is.True);
            Connection.GetMethod("Disconnect").Invoke(connection, null);
            Assert.That(worker.IsCompleted, Is.True);
            Assert.That(worker.IsFaulted, Is.False);
            Assert.That(server.Wait(3000), Is.True);
            Assert.That(Connection.GetProperty("HasConnectionThread").GetValue(connection), Is.False);
        }
        finally { listener.Stop(); UnityEngine.Object.DestroyImmediate(go); }
    }

    private static Task<Tuple<string, byte[]>> Read(object reader, NetworkStream stream)
    {
        return Task.Run(() => (Task<Tuple<string, byte[]>>)reader.GetType().GetMethod("Read")
            .Invoke(reader, new object[] { stream, 1, CancellationToken.None }));
    }
    [Test] public void TruncatedHeaderIsAFailureAndFrameBoundaryEofIsOrderly()
    {
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        try
        {
            for (int size = 0; size <= 2; size += 2)
            {
                using (var client = new TcpClient())
                {
                    client.Connect(IPAddress.Loopback, ((IPEndPoint)listener.LocalEndpoint).Port);
                    using (TcpClient peer = listener.AcceptTcpClient())
                    {
                        if (size > 0) peer.GetStream().Write(new byte[size], 0, size);
                        peer.Client.Shutdown(SocketShutdown.Send);
                        object reader = Activator.CreateInstance(Connection.GetNestedType("MessageReader", BindingFlags.NonPublic));
                        Task<Tuple<string, byte[]>> task = Read(reader, client.GetStream());
                        Assert.That(SpinWait.SpinUntil(() => task.IsCompleted, 3000), Is.True);
                        if (size == 0) Assert.That(task.GetAwaiter().GetResult(), Is.Null);
                        else Assert.Throws<EndOfStreamException>(() => task.GetAwaiter().GetResult());
                    }
                }
            }
        }
        finally { listener.Stop(); }
    }

    [Test] public void ReconnectionPublisherRegistrationUsesTheStreamBeforeQueuedData()
    {
        var go = new GameObject("registration ordering");
        try
        {
            Component connection = go.AddComponent(Connection);
            Type api = Connection.GetNestedType("InternalAPI", BindingFlags.Public | BindingFlags.NonPublic);
            object instance = Activator.CreateInstance(api, new object[] { connection });
            var listener = new TcpListener(IPAddress.Loopback, 0);
            listener.Start();
            try
            {
                using (var sender = new TcpClient())
                {
                    sender.Connect(IPAddress.Loopback, ((IPEndPoint)listener.LocalEndpoint).Port);
                    using (var peer = listener.AcceptTcpClient())
                    {
                        api.GetMethod("SendPublisherRegistration").Invoke(instance,
                            new object[] { "/test", "std_msgs/Bool", 10, false, sender.GetStream() });
                        Assert.That(peer.GetStream().DataAvailable, Is.True);
                        Task<string> frame = Task.Run(() => Frame(peer.GetStream()));
                        Assert.That(frame.Wait(3000), Is.True);
                        Assert.That(frame.Result, Is.EqualTo("__publish"));
                    }
                }
            }
            finally { listener.Stop(); }
        }
        finally { UnityEngine.Object.DestroyImmediate(go); }
    }

    [Test] public void ImmediateDisconnectWaitsForTheCapturedWorker()
    {
        var go = new GameObject("immediate cancellation");
        try
        {
            Component connection = go.AddComponent(Connection);
            for (int i = 0; i < 20; i++)
            {
                Connection.GetMethod("Connect", Type.EmptyTypes).Invoke(connection, null);
                Task worker = (Task)Connection.GetField("m_ConnectionTask", Private).GetValue(connection);
                Connection.GetMethod("Disconnect").Invoke(connection, null);
                Assert.That(worker.IsCompleted, Is.True);
                Assert.That(worker.IsFaulted, Is.False);
                Assert.That(Connection.GetProperty("HasConnectionThread").GetValue(connection), Is.False);
            }
        }
        finally { UnityEngine.Object.DestroyImmediate(go); }
    }

    [Test] public void ConcurrentReadersDoNotShareFrameBuffers()
    {
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        try
        {
            var tasks = new List<Task>();
            foreach (int size in new[] { 8, 256 })
            {
                tasks.Add(Task.Run(async () => {
                    using (var client = new TcpClient())
                    {
                        await client.ConnectAsync(IPAddress.Loopback, ((IPEndPoint)listener.LocalEndpoint).Port);
                        object reader = Activator.CreateInstance(Connection.GetNestedType("MessageReader", BindingFlags.NonPublic));
                        var frame = await Read(reader, client.GetStream());
                        Assert.That(frame.Item2.Length, Is.EqualTo(frame.Item1.Length));
                        Assert.That(frame.Item2.All(value => value == frame.Item1[0]), Is.True);
                    }
                }));
            }
            var peers = new[] { listener.AcceptTcpClient(), listener.AcceptTcpClient() };
            byte[][] wires = {
                Wire(new string('a', 8)+"\0", Enumerable.Repeat((byte)'a', 8).ToArray()),
                Wire(new string('b', 256), Enumerable.Repeat((byte)'b', 256).ToArray())
            };
            for (int index = 0; index < wires.Max(w => w.Length); index++)
                for (int i = 0; i < peers.Length; i++)
                    if (index < wires[i].Length) peers[i].GetStream().WriteByte(wires[i][index]);
            Assert.That(Task.WaitAll(tasks.ToArray(), 3000), Is.True);
            foreach (var peer in peers) peer.Dispose();
        }
        finally { listener.Stop(); }
    }
}
