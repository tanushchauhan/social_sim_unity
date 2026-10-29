using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using SEAN.Scenario.Controllers;

public class OrcaControllerTests
{
    sealed class Sink : ISceneMotionSink
    {
        public MotionTarget[] Targets;
        public void Stop() { Targets = new MotionTarget[0]; }
        public void Commit(IReadOnlyList<MotionTarget> targets) { Targets = targets.ToArray(); }
    }
    static ActorState Person(string id, Vector2 p, Vector2 preferred, Vector2? velocity = null, bool active = true)
    {
        return new ActorState(id, false, active, p, velocity ?? Vector2.zero, 0, .2f, .6f, null, preferred);
    }
    static ActorState Robot(Vector2? p = null, Vector2? v = null)
    {
        return new ActorState("robot", true, true, p ?? new Vector2(20, 20), v ?? Vector2.zero, 0, .4f, 1, null);
    }
    static MotionTarget[] Solve(ActorState[] actors, params Obstacle[] obstacles)
    {
        var sink = new Sink();
        using (var core = new SceneController(new OrcaSceneController(new OrcaSettings()), sink, .1))
        {
            core.Reset("test"); core.Step(0, actors, obstacles);
            return sink.Targets.ToArray();
        }
    }
    static Obstacle Box(string id, float minX, float minY, float maxX, float maxY)
    {
        return new Obstacle(id, new[] { new Vector2(minX,minY), new Vector2(maxX,minY),
            new Vector2(maxX,maxY), new Vector2(minX,maxY) });
    }
    [TestCase(-1)] [TestCase(1)]
    public void MirroredCrossingRemainsFiniteBoundedAndSeparated(int mirror)
    {
        var positions = new[] { new Vector2(-2,0), new Vector2(0,-2*mirror) };
        var goals = new[] { new Vector2(2,0), new Vector2(0,2*mirror) };
        var velocities = new[] { Vector2.zero, Vector2.zero };
        var sink = new Sink();
        using (var core = new SceneController(new OrcaSceneController(new OrcaSettings()), sink, .1))
        {
            core.Reset("crossing");
            for (int n = 0; n < 100; n++)
            {
                var actors = new List<ActorState> { Robot() };
                for (int i = 0; i < 2; i++)
                {
                    var delta = goals[i]-positions[i];
                    actors.Add(Person("p"+i,positions[i],delta.magnitude < .1f ? Vector2.zero : delta.normalized*.6f,velocities[i]));
                }
                core.Step(n*.1, actors, new Obstacle[0]);
                foreach (var target in sink.Targets)
                {
                    int i = target.ActorId == "p0" ? 0 : 1;
                    Assert.True(Contract.Finite(target.Velocity)); Assert.LessOrEqual(target.Velocity.magnitude,.600001f);
                    positions[i] += target.Velocity*.1f; velocities[i] = target.Velocity;
                }
                Assert.GreaterOrEqual(Vector2.Distance(positions[0],positions[1]),.399f);
            }
        }
    }
    [Test] public void ExternalRobotReceivesNoTargetAndPedestrianTakesFullResponsibility()
    {
        var person = Person("person",new Vector2(-1.5f,0),new Vector2(.6f,0));
        var parked = Solve(new[] { person, Robot(Vector2.zero) });
        var moving = Solve(new[] { person, Robot(Vector2.zero,new Vector2(-.3f,0)) });
        Assert.AreEqual(1,parked.Length); Assert.AreEqual("person",parked[0].ActorId);
        Assert.LessOrEqual(parked[0].Velocity.x,.361f);
        Assert.Less(moving[0].Velocity.x,parked[0].Velocity.x);
    }
    [Test] public void WallAndCornerConstrainPreferenceAndDoorwayIsExplicitGeometry()
    {
        var a = Person("person",new Vector2(-1,0),new Vector2(.6f,0));
        var wall = Solve(new[] { a, Robot() },Box("wall",-.2f,-5,.2f,5));
        Assert.LessOrEqual(wall[0].Velocity.x,.241f);
        var doorway = Solve(new[] { a, Robot() },Box("lower",-.2f,-5,.2f,-.6f),Box("upper",-.2f,.6f,.2f,5));
        Assert.Greater(doorway[0].Velocity.x,wall[0].Velocity.x);
        var corner = Solve(new[] { a, Robot() },Box("vertical",-.2f,-5,.2f,5),Box("horizontal",-5,1,0,1.2f));
        Assert.True(Contract.Finite(corner[0].Velocity));
    }
    [Test] public void HeadOnAndStandingPersonProduceBoundedVelocities()
    {
        foreach (float speed in new[] { 0f,-.6f })
        {
            var result=Solve(new[] { Person("a",new Vector2(-1,0),new Vector2(.6f,0)),
                Person("b",new Vector2(1,0),new Vector2(speed,0)),Robot() });
            Assert.AreEqual(2,result.Length);
            Assert.True(result.All(t => Contract.Finite(t.Velocity) && t.Velocity.magnitude<=.600001f));
        }
    }
    [Test] public void ActorPermutationResetAndPopulationChangesCannotLeakSolverState()
    {
        var actors=new[] { Person("a",new Vector2(-1,0),new Vector2(.6f,0)),
            Person("b",new Vector2(1,0),new Vector2(-.6f,0)),Robot() };
        var first=Solve(actors); var reversed=Solve(actors.Reverse().ToArray());
        CollectionAssert.AreEqual(first.Select(t=>t.ActorId),reversed.Select(t=>t.ActorId));
        CollectionAssert.AreEqual(first.Select(t=>t.Velocity),reversed.Select(t=>t.Velocity));
        Assert.AreEqual(1,Solve(new[] { actors[0],Robot() }).Length);
        var again=Solve(actors);
        CollectionAssert.AreEqual(first.Select(t=>t.Velocity),again.Select(t=>t.Velocity));
    }
    [Test] public void ExactlyCoincidentActorsFailExplicitlyInsteadOfReturningNaN()
    {
        Assert.Throws<InvalidOperationException>(()=>Solve(new[] {
            Person("a",Vector2.zero,Vector2.zero),Person("b",Vector2.zero,Vector2.zero),Robot() }));
    }
}
