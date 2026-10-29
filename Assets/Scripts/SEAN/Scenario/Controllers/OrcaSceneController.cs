using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace SEAN.Scenario.Controllers
{
    [Serializable]
    public sealed class OrcaSettings
    {
        public double decisionPeriod = .1;
        public float neighborDistance = 5;
        public int maxNeighbors = 64;
        public float agentHorizon = 2.5f;
        public float obstacleHorizon = 2.5f;
        public void Validate()
        {
            if (!Contract.Finite(decisionPeriod) || decisionPeriod <= 0 ||
                !Contract.Finite(neighborDistance) || neighborDistance <= 0 || maxNeighbors < 1 ||
                !Contract.Finite(agentHorizon) || agentHorizon <= 0 ||
                !Contract.Finite(obstacleHorizon) || obstacleHorizon <= 0)
                throw new ArgumentException("Invalid ORCA parameters");
        }
    }

    public sealed class OrcaSceneController : ISceneController
    {
        // The official C# implementation has a singleton world. Serialize access
        // and reconstruct it from each immutable snapshot so instances cannot
        // inherit one another's state or integrate predictions twice.
        static readonly object SolverLock = new object();
        readonly OrcaSettings settings;
        bool disposed;
        public OrcaSceneController(OrcaSettings settings)
        {
            if (settings == null) throw new ArgumentNullException("settings");
            settings.Validate();
            // Own an immutable-by-convention copy rather than caller-owned knobs.
            this.settings = new OrcaSettings { decisionPeriod = settings.decisionPeriod,
                neighborDistance = settings.neighborDistance, maxNeighbors = settings.maxNeighbors,
                agentHorizon = settings.agentHorizon, obstacleHorizon = settings.obstacleHorizon };
        }
        public void Reset(string episodeId)
        {
            if (disposed) throw new ObjectDisposedException("OrcaSceneController");
        }
        public void Dispose() { disposed = true; }
        static RVO.Vector2 ToRvo(Vector2 v) { return new RVO.Vector2(v.x, v.y); }

        public SceneDecision Decide(SceneSnapshot snapshot)
        {
            if (disposed) throw new ObjectDisposedException("OrcaSceneController");
            lock (SolverLock)
            {
                var solver = RVO.Simulator.Instance;
                solver.Clear(); solver.setTimeStep((float)settings.decisionPeriod);
                foreach (var obstacle in snapshot.Obstacles)
                    solver.addObstacle(obstacle.Vertices.Select(ToRvo).ToList());
                solver.processObstacles();
                var actors = snapshot.Actors.ToArray();
                for (int i = 0; i < actors.Length; i++)
                {
                    var actor = actors[i];
                    for (int j = 0; j < i; j++)
                        if ((actor.Position - actors[j].Position).sqrMagnitude < 1e-12f)
                            throw new InvalidOperationException("ORCA cannot initialize exactly co-located actor centres");
                    solver.addAgent(ToRvo(actor.Position), settings.neighborDistance, settings.maxNeighbors,
                        settings.agentHorizon, settings.obstacleHorizon, actor.Radius,
                        actor.IsRobot || !actor.Active ? actor.Velocity.magnitude : actor.MaxSpeed, ToRvo(actor.Velocity));
                    solver.setAgentExternalControl(i, actor.IsRobot || !actor.Active);
                    solver.setAgentPrefVelocity(i, ToRvo(actor.IsRobot || !actor.Active ? actor.Velocity :
                        actor.PreferredVelocity ?? Vector2.zero));
                }
                // Exactly one joint solver step. Its integrated positions are
                // never applied; Unity executes targets and provides feedback.
                solver.doStep();
                var targets = new List<MotionTarget>();
                for (int i = 0; i < actors.Length; i++)
                {
                    if (actors[i].IsRobot || !actors[i].Active) continue;
                    var velocity = solver.getAgentVelocity(i);
                    var value = new Vector2(velocity.x(), velocity.y());
                    if (!Contract.Finite(value) || value.magnitude > actors[i].MaxSpeed + 1e-6f)
                        throw new InvalidOperationException("ORCA produced an invalid velocity");
                    targets.Add(new MotionTarget(actors[i].Id, value));
                }
                return new SceneDecision(snapshot.EpisodeId, snapshot.Sequence, snapshot.SimulationTime,
                    snapshot.SimulationTime + settings.decisionPeriod, targets);
            }
        }
    }
}
