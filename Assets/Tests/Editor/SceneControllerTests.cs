using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using SEAN.Scenario.Controllers;

public class SceneControllerTests
{
    sealed class FakeController : ISceneController
    {
        public int Calls, Resets;
        public bool Broken;
        public SceneSnapshot Last;
        public void Reset(string id) { Resets++; Last = null; }
        public void Dispose() { Last = null; }
        public SceneDecision Decide(SceneSnapshot snapshot)
        {
            Calls++; Last = snapshot;
            if (Broken) throw new InvalidOperationException("test worker failure");
            return Decision(snapshot, snapshot.Actors.Where(a => a.Active && !a.IsRobot)
                .Select(a => new MotionTarget(a.Id, new Vector2(.25f, 0))).ToArray());
        }
    }
    sealed class Sink : ISceneMotionSink
    {
        public readonly Dictionary<string, Vector2> Velocities = new Dictionary<string, Vector2>();
        public readonly Dictionary<string, Vector2> Positions = new Dictionary<string, Vector2>();
        public int Commits;
        public void Commit(IReadOnlyList<MotionTarget> targets)
        {
            Commits++;
            Velocities.Clear();
            foreach (var target in targets) Velocities[target.ActorId] = target.Velocity;
        }
        public void Stop() { Velocities.Clear(); }
        public void Integrate(float dt)
        {
            foreach (var pair in Velocities) Positions[pair.Key] += pair.Value * dt;
        }
    }
    FakeController worker;
    Sink sink;
    SceneController core;
    static readonly Obstacle[] NoObstacles = new Obstacle[0];
    static ActorState Actor(string id, bool robot = false, bool active = true, Vector2? position = null)
    {
        return new ActorState(id, robot, active, position ?? Vector2.zero,
            Vector2.zero, 0, .2f, .6f, new Vector2(3, 0));
    }
    static ActorState[] Actors() { return new[] { Actor("b"), Actor("robot", true), Actor("a") }; }
    static SceneDecision Decision(SceneSnapshot s, params MotionTarget[] targets)
    {
        return new SceneDecision(s.EpisodeId, s.Sequence, s.SimulationTime, s.SimulationTime + .4, targets);
    }
    [SetUp] public void Setup()
    {
        worker = new FakeController(); sink = new Sink();
        core = new SceneController(worker, sink, .4); core.Reset("episode-1");
    }
    [TearDown] public void Teardown() { core.Dispose(); }

    [Test] public void OneJointCallAndAtomicBatchAreIndependentOfActorOrder()
    {
        core.Step(0, Actors(), NoObstacles);
        Assert.AreEqual(1, worker.Calls); Assert.AreEqual(1, sink.Commits);
        CollectionAssert.AreEqual(new[] { "a", "b", "robot" }, worker.Last.Actors.Select(a => a.Id));
        Assert.False(sink.Velocities.ContainsKey("robot"));
        var first = sink.Velocities.ToArray();
        core.Reset("episode-2"); core.Step(0, Actors().Reverse(), NoObstacles);
        CollectionAssert.AreEquivalent(first, sink.Velocities.ToArray());
    }
    [Test] public void ActualExecutedPositionsFeedTheNextJointSnapshot()
    {
        sink.Positions["a"] = Vector2.zero; sink.Positions["b"] = Vector2.zero;
        core.Step(0, Actors(), NoObstacles);
        sink.Integrate(.2f); // fixture executes half the decision horizon
        core.Tick(.2);
        core.Step(.2, new[] { Actor("a", position: sink.Positions["a"]),
            Actor("b", position: sink.Positions["b"]), Actor("robot", true) }, NoObstacles);
        Assert.AreEqual(.05f, worker.Last.Actors[0].Position.x, 1e-6);
        Assert.AreEqual(1, worker.Last.Sequence);
    }
    [Test] public void ResetClearsVelocityHistoryAndRejectsOldEpisode()
    {
        core.Step(0, Actors(), NoObstacles); var old = worker.Last;
        core.Reset("episode-2"); Assert.AreEqual(0, sink.Velocities.Count); Assert.IsNull(worker.Last);
        var current = core.BeginStep(0, Actors(), NoObstacles);
        Assert.False(core.Apply(Decision(old, new MotionTarget("a", Vector2.zero), new MotionTarget("b", Vector2.zero))));
        Assert.True(core.Apply(worker.Decide(current)));
        Assert.AreEqual(2, worker.Resets);
        Assert.Throws<ArgumentException>(() => core.Reset("episode-1"));
    }
    [TestCase("robot")]
    [TestCase("duplicate")]
    [TestCase("missing")]
    [TestCase("nan")]
    [TestCase("speed")]
    [TestCase("sequence")]
    [TestCase("horizon")]
    public void InvalidOutputCannotPartiallyCommit(string kind)
    {
        var s = core.BeginStep(0, Actors(), NoObstacles);
        var targets = new[] { new MotionTarget("a", Vector2.zero), new MotionTarget("b", Vector2.zero) };
        if (kind == "robot") targets[1] = new MotionTarget("robot", Vector2.zero);
        if (kind == "duplicate") targets[1] = targets[0];
        if (kind == "missing") targets = new MotionTarget[0];
        if (kind == "nan") targets[1] = new MotionTarget("b", new Vector2(float.NaN, 0));
        if (kind == "speed") targets[1] = new MotionTarget("b", new Vector2(1, 0));
        var d = new SceneDecision(s.EpisodeId, kind == "sequence" ? 4 : s.Sequence, 0,
            kind == "horizon" ? .8 : .4, targets);
        Assert.False(core.Apply(d)); Assert.AreEqual(0, sink.Commits);
        Assert.True(core.Apply(worker.Decide(s)));
        Assert.False(core.Apply(worker.Decide(s)), "A response may only commit once");
    }
    [Test] public void WorkerFailureStopsMotionAndRequiresReset()
    {
        core.Step(0, Actors(), NoObstacles); worker.Broken = true;
        Assert.Throws<InvalidOperationException>(() => core.Step(.2, Actors(), NoObstacles));
        Assert.AreEqual(0, sink.Velocities.Count);
        Assert.Throws<InvalidOperationException>(() => core.BeginStep(.4, Actors(), NoObstacles));
        core.Reset("episode-2"); worker.Broken = false; core.Step(0, Actors(), NoObstacles);
    }
    [Test] public void ValidityExpiryAndDisposalStopAllMotion()
    {
        core.Step(0, Actors(), NoObstacles); core.Tick(.399);
        Assert.AreEqual(2, sink.Velocities.Count);
        core.Tick(.4); Assert.AreEqual(0, sink.Velocities.Count);
        core.Step(.4, Actors(), NoObstacles); core.Dispose();
        Assert.AreEqual(0, sink.Velocities.Count);
        Assert.Throws<ObjectDisposedException>(() => core.Step(.6, Actors(), NoObstacles));
    }
    [Test] public void PendingDecisionCannotOutliveSimulationBoundary()
    {
        core.Step(0, Actors(), NoObstacles);
        core.BeginStep(.2, Actors(), NoObstacles);
        Assert.Throws<InvalidOperationException>(() => core.Tick(.21));
        Assert.AreEqual(0, sink.Velocities.Count);
    }
    [Test] public void RegressingClockStopsMotion()
    {
        core.Step(1, Actors(), NoObstacles);
        Assert.Throws<ArgumentException>(() => core.Tick(.9));
        Assert.AreEqual(0, sink.Velocities.Count);
        Assert.Throws<InvalidOperationException>(() => core.Tick(1.1));
    }
    [Test] public void DuplicateIdsAndNonfiniteInputsAreRejected()
    {
        Assert.Throws<ArgumentException>(() => Actor("", position: Vector2.zero));
        Assert.Throws<ArgumentException>(() => Actor("x", position: new Vector2(float.PositiveInfinity, 0)));
        Assert.Throws<ArgumentException>(() => core.BeginStep(0, new[] { Actor("robot", true), Actor("x"), Actor("x") }, NoObstacles));
        Assert.Throws<ArgumentException>(() => core.BeginStep(double.NaN, Actors(), NoObstacles));
    }
    [Test] public void InactiveActorsAreContextOnly()
    {
        core.Step(0, new[] { Actor("robot", true), Actor("standing", active: false) }, NoObstacles);
        Assert.AreEqual(0, sink.Velocities.Count);
    }
    [TestCase(0f)] [TestCase(1.5707963f)] [TestCase(-2.4f)]
    public void CoordinateAndHeadingRoundTrip(float yaw)
    {
        var forward = new Vector3(Mathf.Sin(yaw), 0, Mathf.Cos(yaw));
        var planar = Contract.FromUnity(forward);
        Assert.Less((Contract.ToUnity(planar) - forward).magnitude, 1e-6);
        Assert.AreEqual(Contract.HeadingFromUnity(yaw), Mathf.Atan2(planar.y, planar.x), 1e-6);
    }
}
