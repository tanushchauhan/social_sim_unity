using System;
using System.Diagnostics;
using System.IO;
using System.Globalization;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace SEAN.Scenario.Controllers
{
    // Synchronous local worker. Blocking the simulation thread prevents physics
    // advancing while inference is pending. SceneController/driver owns stopping
    // on any exception and validates the complete returned target batch.
    public sealed class SocketSceneController : ISceneController
    {
        const int MaxPacket = 4 * 1024 * 1024;
        readonly TcpClient client;
        readonly NetworkStream stream;
        readonly int timeoutMs, observationSteps;
        readonly double dt;
        bool disposed, faulted;
        string episode;
        public bool HistoryReady { get; private set; }

        [Serializable] sealed class ActorPacket
        {
            public string id;
            public bool robot, active, has_destination, has_preferred_velocity;
            public float[] position, velocity, destination, preferred_velocity;
            public float heading, radius, max_speed;
        }
        [Serializable] sealed class ObstaclePacket { public string id; public PointPacket[] vertices; }
        [Serializable] sealed class PointPacket { public float x, y; }
        [Serializable] sealed class Request
        {
            public int version = 1, observation_steps;
            public string kind, episode, time_token;
            public long sequence;
            public double time, dt;
            public ActorPacket[] actors;
            public ObstaclePacket[] obstacles;
        }
        [Serializable] sealed class TargetPacket { public string id; public float[] velocity; }
        [Serializable] sealed class Response
        {
            public int version;
            public string kind, episode, time_token, valid_until_token;
            public long sequence;
            public double time, valid_until;
            public bool ready;
            public TargetPacket[] targets;
        }

        public SocketSceneController(int port, int observationSteps, double dt, int timeoutMs = 5000)
        {
            if (port < 1 || port > 65535 || observationSteps < 2 || !Contract.Finite(dt) || dt <= 0 || timeoutMs <= 0)
                throw new ArgumentException("Valid local endpoint, history/cadence and bounded wall timeout required");
            this.observationSteps = observationSteps; this.dt = dt; this.timeoutMs = timeoutMs;
            client = new TcpClient { NoDelay = true };
            try
            {
                var pending = client.ConnectAsync(IPAddress.Loopback, port);
                if (!pending.Wait(timeoutMs)) throw new TimeoutException("Worker connect deadline exceeded");
                pending.GetAwaiter().GetResult();
                stream = client.GetStream();
            }
            catch { client.Close(); throw; }
        }

        Response Exchange(Request request)
        {
            if (disposed || faulted) throw new InvalidOperationException("Worker connection is closed or faulted");
            try
            {
                var timer = Stopwatch.StartNew();
                var body = Encoding.UTF8.GetBytes(JsonUtility.ToJson(request));
                if (body.Length > MaxPacket) throw new InvalidDataException("Worker packet too large");
                var size = BitConverter.GetBytes(IPAddress.HostToNetworkOrder(body.Length));
                stream.WriteTimeout = timeoutMs;
                stream.Write(size, 0, 4);
                stream.WriteTimeout = Remaining(timer);
                stream.Write(body, 0, body.Length);
                var header = Read(4, timer);
                int count = IPAddress.NetworkToHostOrder(BitConverter.ToInt32(header, 0));
                if (count < 1 || count > MaxPacket) throw new InvalidDataException("Invalid worker packet length");
                var response = JsonUtility.FromJson<Response>(Encoding.UTF8.GetString(Read(count, timer)));
                if (response == null || response.version != 1 || response.episode != request.episode)
                    throw new InvalidDataException("Wrong worker protocol or episode");
                return response;
            }
            catch { faulted = true; client.Close(); throw; }
        }
        int Remaining(Stopwatch timer)
        {
            long remaining = timeoutMs - timer.ElapsedMilliseconds;
            if (remaining <= 0) throw new TimeoutException("Worker exchange deadline exceeded");
            return (int)remaining;
        }
        byte[] Read(int count, Stopwatch timer)
        {
            var bytes = new byte[count];
            for (int offset = 0; offset < count;)
            {
                stream.ReadTimeout = Remaining(timer);
                int received = stream.Read(bytes, offset, count - offset);
                if (received == 0) throw new EndOfStreamException("Worker disconnected");
                offset += received;
            }
            return bytes;
        }
        public void Reset(string episodeId)
        {
            HistoryReady = false;
            var response = Exchange(new Request { kind = "reset", episode = episodeId,
                observation_steps = observationSteps, dt = dt });
            if (response.kind != "reset") { faulted = true; throw new InvalidDataException("Worker did not acknowledge reset"); }
            episode = episodeId;
        }
        public SceneDecision Decide(SceneSnapshot snapshot)
        {
            if (snapshot.EpisodeId != episode) throw new InvalidOperationException("Reset the worker before stepping");
            var response = Exchange(new Request { kind = "step", episode = episode,
                sequence = snapshot.Sequence, time = snapshot.SimulationTime,
                time_token = snapshot.SimulationTime.ToString("R", CultureInfo.InvariantCulture),
                actors = snapshot.Actors.Select(a => new ActorPacket {
                    id = a.Id, robot = a.IsRobot, active = a.Active, position = Pair(a.Position), velocity = Pair(a.Velocity),
                    heading = a.Heading, radius = a.Radius, max_speed = a.MaxSpeed,
                    has_destination = a.Destination.HasValue, destination = Pair(a.Destination ?? Vector2.zero),
                    has_preferred_velocity = a.PreferredVelocity.HasValue,
                    preferred_velocity = Pair(a.PreferredVelocity ?? Vector2.zero)
                }).ToArray(),
                obstacles = snapshot.Obstacles.Select(o => new ObstaclePacket { id = o.Id,
                    vertices = o.Vertices.Select(v => new PointPacket { x = v.x, y = v.y }).ToArray() }).ToArray() });
            double endpoint;
            if (response.kind != "decision" || response.sequence != snapshot.Sequence ||
                response.time_token != snapshot.SimulationTime.ToString("R", CultureInfo.InvariantCulture) ||
                !Contract.Finite(response.time) || Math.Abs(response.time - snapshot.SimulationTime) > 1e-6 ||
                !double.TryParse(response.valid_until_token, NumberStyles.Float, CultureInfo.InvariantCulture, out endpoint) ||
                !Contract.Finite(endpoint) || Math.Abs(endpoint - snapshot.SimulationTime - dt) > 1e-9 ||
                !Contract.Finite(response.valid_until) || Math.Abs(response.valid_until - endpoint) > 1e-6 ||
                response.targets == null || response.targets.Any(t => t == null || t.velocity == null || t.velocity.Length != 2))
            { faulted = true; throw new InvalidDataException("Malformed or stale worker response"); }
            HistoryReady = response.ready;
            return new SceneDecision(response.episode, response.sequence, snapshot.SimulationTime, endpoint,
                response.targets.Select(t => new MotionTarget(t.id, new Vector2(t.velocity[0], t.velocity[1]))));
        }
        static float[] Pair(Vector2 value) { return new[] { value.x, value.y }; }
        public void Dispose()
        {
            if (disposed) return;
            disposed = true; client.Close();
        }
    }
}
