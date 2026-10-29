using System;
using System.Linq;
using UnityEngine;

namespace SEAN.Scenario.Controllers
{
    // Explicit test double: follows the supplied path preference with no local
    // avoidance. Never report this controller as ORCA or STARS.
    public sealed class GoalTestController : ISceneController
    {
        readonly double horizon;
        public GoalTestController(double horizon)
        {
            if (!Contract.Finite(horizon) || horizon <= 0) throw new ArgumentException("Invalid horizon");
            this.horizon = horizon;
        }
        public void Reset(string episodeId) { }
        public void Dispose() { }
        public SceneDecision Decide(SceneSnapshot snapshot)
        {
            return new SceneDecision(snapshot.EpisodeId, snapshot.Sequence,
                snapshot.SimulationTime, snapshot.SimulationTime + horizon,
                snapshot.Actors.Where(a => a.Active && !a.IsRobot)
                    .Select(a => new MotionTarget(a.Id, a.PreferredVelocity ?? Vector2.zero)));
        }
    }
}
