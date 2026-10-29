using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using NUnit.Framework;
using UnityEngine;
using SEAN.Scenario.Controllers;

public class SocketSceneControllerTests
{
    sealed class Sink : ISceneMotionSink
    {
        public bool Moving;
        public void Commit(IReadOnlyList<MotionTarget> targets) { Moving = targets.Count > 0 && targets[0].Velocity.magnitude > 0; }
        public void Stop() { Moving = false; }
    }
    static ActorState[] Actors(float actualPosition = 0)
    {
        return new[] {
            new ActorState("person", false, true, new Vector2(actualPosition, 0), Vector2.zero, 0, .2f, .6f, null),
            new ActorState("robot", true, true, new Vector2(2, 0), Vector2.zero, 0, .25f, 1, null)
        };
    }
    [Serializable] sealed class Startup { public int port; }

    [Test] public void PythonWorkerWarmsUpFromActualStatesAndResetsAcrossEpisodes()
    {
        string script = Environment.GetEnvironmentVariable("SEAN_WORKER_SCRIPT");
        if (string.IsNullOrEmpty(script)) Assert.Ignore("Set SEAN_WORKER_SCRIPT to workspace runtime/stars_bridge.py for cross-process validation");
        var start = new ProcessStartInfo(Environment.GetEnvironmentVariable("SEAN_WORKER_PYTHON") ?? "python3",
            "-u \"" + script + "\" --test-double --port 0 --observation-steps 3 --dt .4") {
            UseShellExecute = false, RedirectStandardOutput = true, RedirectStandardError = true, CreateNoWindow = true
        };
        using (var process = Process.Start(start))
        {
            try
            {
                var startup = process.StandardOutput.ReadLineAsync();
                Assert.True(startup.Wait(5000), "Python worker startup timeout");
                Assert.NotNull(startup.Result, "Python worker failed to start");
                int port = JsonUtility.FromJson<Startup>(startup.Result).port;
                var worker = new SocketSceneController(port, 3, .4);
                using (var core = new SceneController(worker, new Sink(), .4))
                {
                    core.Reset("first");
                    core.Step(0, Actors(), new Obstacle[0]); Assert.False(worker.HistoryReady);
                    core.Step(.4, Actors(1), new Obstacle[0]); Assert.False(worker.HistoryReady);
                    core.Step(.8, Actors(2), new Obstacle[0]); Assert.True(worker.HistoryReady);
                    core.Reset("second");
                    core.Step(0, Actors(), new Obstacle[0]); Assert.False(worker.HistoryReady);
                }
                Assert.True(process.WaitForExit(5000), "Worker did not exit on disconnect");
                Assert.AreEqual(0, process.ExitCode, process.StandardError.ReadToEnd());
            }
            finally { if (!process.HasExited) { process.Kill(); process.WaitForExit(); } }
        }
    }

    static void ReadRequest(NetworkStream stream)
    {
        var header = Read(stream, 4);
        int size = IPAddress.NetworkToHostOrder(BitConverter.ToInt32(header, 0));
        if (size < 1 || size > 4 * 1024 * 1024) throw new IOException("Bad test request");
        Read(stream, size);
    }
    static byte[] Read(NetworkStream stream, int count)
    {
        var result = new byte[count];
        for (int offset = 0; offset < count;)
        {
            int got = stream.Read(result, offset, count - offset);
            if (got == 0) throw new EndOfStreamException();
            offset += got;
        }
        return result;
    }
    static void Write(NetworkStream stream, string json)
    {
        byte[] body = Encoding.UTF8.GetBytes(json);
        byte[] header = BitConverter.GetBytes(IPAddress.HostToNetworkOrder(body.Length));
        stream.Write(header, 0, 4); stream.Write(body, 0, body.Length);
    }

    [TestCase("timeout")]
    [TestCase("disconnect")]
    [TestCase("stale")]
    [TestCase("overspeed")]
    [TestCase("robot")]
    public void WorkerFailureStopsTheWholeBatch(string fault)
    {
        var listener = new TcpListener(IPAddress.Loopback, 0); listener.Start();
        int port = ((IPEndPoint)listener.LocalEndpoint).Port;
        var server = Task.Run(() => {
            using (var peer = listener.AcceptTcpClient())
            {
                var stream = peer.GetStream(); stream.ReadTimeout = 3000;
                ReadRequest(stream);
                Write(stream, "{\"version\":1,\"kind\":\"reset\",\"episode\":\"e\"}");
                ReadRequest(stream);
                if (fault == "timeout") { Thread.Sleep(500); return; }
                if (fault == "disconnect") return;
                Write(stream, "{\"version\":1,\"kind\":\"decision\",\"episode\":\"e\",\"sequence\":" +
                    (fault == "stale" ? "9" : "0") + ",\"time\":0,\"valid_until\":0.4,\"ready\":true,\"targets\":[{\"id\":\"" +
                    (fault == "robot" ? "robot" : "person") + "\",\"velocity\":[" + (fault == "overspeed" ? "5" : "0.2") + ",0]}]}");
            }
        });
        try
        {
            var sink = new Sink();
            using (var core = new SceneController(new SocketSceneController(port, 3, .4, 200), sink, .4))
            {
                core.Reset("e"); sink.Moving = true;
                var timer = Stopwatch.StartNew();
                Assert.Catch(() => core.Step(0, Actors(), new Obstacle[0]));
                Assert.False(sink.Moving, "Worker loss left a prior velocity active");
                Assert.Less(timer.Elapsed.TotalSeconds, 2, "Failure exceeded wall bound");
            }
            Assert.True(server.Wait(3000));
        }
        finally { listener.Stop(); }
    }
}
