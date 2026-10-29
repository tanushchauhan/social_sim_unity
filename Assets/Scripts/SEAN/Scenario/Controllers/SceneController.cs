using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace SEAN.Scenario.Controllers
{
    // All planar values use ROS map FLU: x forward, y left, metres, seconds,
    // radians counterclockwise from +x. Unity (x,y,z) -> ROS (z,-x,y).
    public sealed class ActorState
    {
        public readonly string Id;
        public readonly bool IsRobot, Active;
        public readonly Vector2 Position, Velocity;
        public readonly float Heading, Radius, MaxSpeed;
        public readonly Vector2? Destination;
        public readonly Vector2? PreferredVelocity;
        public ActorState(string id, bool isRobot, bool active, Vector2 position,
            Vector2 velocity, float heading, float radius, float maxSpeed, Vector2? destination,
            Vector2? preferredVelocity = null)
        {
            if (string.IsNullOrWhiteSpace(id) || !Contract.Finite(position) ||
                !Contract.Finite(velocity) || !Contract.Finite(heading) ||
                !Contract.Finite(radius) || radius <= 0 || !Contract.Finite(maxSpeed) || maxSpeed < 0 ||
                (destination.HasValue && !Contract.Finite(destination.Value)) ||
                (preferredVelocity.HasValue && (!Contract.Finite(preferredVelocity.Value) ||
                    preferredVelocity.Value.magnitude > maxSpeed + 1e-6f)))
                throw new ArgumentException("Invalid actor state");
            Id = id; IsRobot = isRobot; Active = active; Position = position;
            Velocity = velocity; Heading = heading; Radius = radius; MaxSpeed = maxSpeed;
            Destination = destination;
            PreferredVelocity = preferredVelocity;
        }
    }

    public sealed class Obstacle
    {
        public readonly string Id;
        public readonly IReadOnlyList<Vector2> Vertices;
        public Obstacle(string id, IEnumerable<Vector2> vertices)
        {
            var copy = vertices.ToArray();
            if (string.IsNullOrWhiteSpace(id) || copy.Length < 3 || copy.Any(v => !Contract.Finite(v)))
                throw new ArgumentException("Invalid obstacle polygon");
            Id = id; Vertices = Array.AsReadOnly(copy);
        }
    }

    public sealed class SceneSnapshot
    {
        public readonly string EpisodeId;
        public readonly long Sequence;
        public readonly double SimulationTime;
        public readonly IReadOnlyList<ActorState> Actors;
        public readonly IReadOnlyList<Obstacle> Obstacles;
        internal SceneSnapshot(string episode, long sequence, double time,
            IEnumerable<ActorState> actors, IEnumerable<Obstacle> obstacles)
        {
            var copy = actors.OrderBy(a => a.Id, StringComparer.Ordinal).ToArray();
            if (copy.Select(a => a.Id).Distinct(StringComparer.Ordinal).Count() != copy.Length ||
                copy.Count(a => a.IsRobot) != 1)
                throw new ArgumentException("Snapshot needs unique actor IDs and exactly one robot");
            EpisodeId = episode; Sequence = sequence; SimulationTime = time;
            Actors = Array.AsReadOnly(copy);
            Obstacles = Array.AsReadOnly(obstacles.ToArray());
        }
    }

    public sealed class MotionTarget
    {
        public readonly string ActorId;
        public readonly Vector2 Velocity;
        public MotionTarget(string actorId, Vector2 velocity) { ActorId = actorId; Velocity = velocity; }
    }

    public sealed class SceneDecision
    {
        public readonly string EpisodeId;
        public readonly long SourceSequence;
        public readonly double SimulationTime, ValidUntil;
        public readonly IReadOnlyList<MotionTarget> Targets;
        public SceneDecision(string episode, long sequence, double time, double validUntil,
            IEnumerable<MotionTarget> targets)
        {
            EpisodeId = episode; SourceSequence = sequence;
            SimulationTime = time; ValidUntil = validUntil;
            Targets = Array.AsReadOnly(targets.ToArray());
        }
    }

    // Implementations own histories/latent state and clear them on Reset/Dispose.
    public interface ISceneController : IDisposable
    {
        void Reset(string episodeId);
        SceneDecision Decide(SceneSnapshot snapshot);
    }

    // Calls occur on Unity's simulation thread, before advancing physics.
    // Commit must replace the entire target batch without advancing any actor.
    // Stop clears animation targets AND achieved rigidbody velocity for pedestrians.
    public interface ISceneMotionSink
    {
        void Commit(IReadOnlyList<MotionTarget> targets);
        void Stop();
    }

    public static class Contract
    {
        public static bool Finite(double v) { return !double.IsNaN(v) && !double.IsInfinity(v); }
        public static bool Finite(Vector2 v) { return Finite(v.x) && Finite(v.y); }
        public static Vector2 FromUnity(Vector3 p) { return new Vector2(p.z, -p.x); }
        public static Vector3 ToUnity(Vector2 p) { return new Vector3(-p.y, 0, p.x); }
        public static float HeadingFromUnity(float yawRadians) { return -yawRadians; }
    }

    // Transport-independent boundary core. It is deliberately not registered in
    // production scenes: SF retains its existing Update/root-motion path.
    public sealed class SceneController : IDisposable
    {
        readonly ISceneController controller;
        readonly ISceneMotionSink sink;
        readonly double maxHorizon;
        string episode;
        long sequence;
        double lastTime, lastDecisionTime, validUntil = double.NegativeInfinity;
        SceneSnapshot pending;
        bool disposed, faulted;
        readonly HashSet<string> usedEpisodes = new HashSet<string>(StringComparer.Ordinal);
        public string EpisodeId { get { return episode; } }

        public SceneController(ISceneController controller, ISceneMotionSink sink, double maxHorizon)
        {
            if (controller == null || sink == null || !Contract.Finite(maxHorizon) || maxHorizon <= 0)
                throw new ArgumentException("Controller, sink and positive horizon required");
            this.controller = controller; this.sink = sink; this.maxHorizon = maxHorizon;
        }

        public void Reset(string episodeId)
        {
            if (disposed) throw new ObjectDisposedException("SceneController");
            if (string.IsNullOrWhiteSpace(episodeId) || !usedEpisodes.Add(episodeId))
                throw new ArgumentException("Every reset requires a fresh episode ID");
            pending = null; validUntil = double.NegativeInfinity; episode = null; faulted = true;
            sink.Stop(); // stop before resetting a worker which may fail
            controller.Reset(episodeId);
            episode = episodeId; sequence = 0;
            lastTime = lastDecisionTime = double.NegativeInfinity; faulted = false;
        }

        public SceneSnapshot BeginStep(double time, IEnumerable<ActorState> actors, IEnumerable<Obstacle> obstacles)
        {
            Ready();
            if (pending != null) throw new InvalidOperationException("Decision pending; do not advance simulation");
            if (!Contract.Finite(time) || time < 0 || time < lastTime || time <= lastDecisionTime)
                throw new ArgumentException("Decision time must strictly increase within an episode");
            Tick(time);
            var snapshot = new SceneSnapshot(episode, sequence++, time, actors, obstacles);
            pending = snapshot; lastTime = lastDecisionTime = time;
            return snapshot;
        }

        // A rejected packet never consumes a pending snapshot or changes targets.
        // Caller may wait for the correct response while physics remains paused.
        public bool Apply(SceneDecision decision)
        {
            Ready();
            if (pending == null || decision == null || decision.EpisodeId != episode ||
                decision.SourceSequence != pending.Sequence || decision.SimulationTime != pending.SimulationTime ||
                !Contract.Finite(decision.ValidUntil) || decision.ValidUntil <= lastTime ||
                decision.ValidUntil - lastTime > maxHorizon + 1e-9) return false;
            var expected = pending.Actors.Where(a => a.Active && !a.IsRobot).ToDictionary(a => a.Id);
            if (decision.Targets.Count != expected.Count) return false;
            var seen = new HashSet<string>(StringComparer.Ordinal);
            foreach (var target in decision.Targets)
            {
                ActorState actor;
                if (target == null || target.ActorId == null || !seen.Add(target.ActorId) ||
                    !expected.TryGetValue(target.ActorId, out actor) || !Contract.Finite(target.Velocity) ||
                    target.Velocity.magnitude > actor.MaxSpeed + 1e-6f) return false;
            }
            try { sink.Commit(decision.Targets); }
            catch { Fail(); throw; }
            validUntil = decision.ValidUntil; pending = null;
            return true;
        }

        public void Step(double time, IEnumerable<ActorState> actors, IEnumerable<Obstacle> obstacles)
        {
            try
            {
                var snapshot = BeginStep(time, actors, obstacles);
                if (!Apply(controller.Decide(snapshot)))
                    throw new InvalidOperationException("Controller returned an invalid decision");
            }
            catch { Fail(); throw; }
        }

        public void Tick(double time)
        {
            Ready();
            if (!Contract.Finite(time) || time < lastTime)
            {
                Fail();
                throw new ArgumentException("Clock regressed or became nonfinite");
            }
            if (pending != null && time > pending.SimulationTime)
            {
                Fail();
                throw new InvalidOperationException("Physics advanced while a decision was pending");
            }
            if (time >= validUntil) { sink.Stop(); validUntil = double.NegativeInfinity; }
            lastTime = time;
        }

        public void Fail()
        {
            pending = null; validUntil = double.NegativeInfinity; faulted = true;
            sink.Stop();
        }

        void Ready()
        {
            if (disposed) throw new ObjectDisposedException("SceneController");
            if (episode == null || faulted) throw new InvalidOperationException("Reset required before stepping");
        }

        public void Dispose()
        {
            if (disposed) return;
            disposed = true;
            try { sink.Stop(); }
            finally { controller.Dispose(); }
        }
    }
}
