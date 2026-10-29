using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using SEAN.Scenario.Controllers;

namespace SEAN.Scenario.Agents
{
    // Runs before AnimatePhysics; one snapshot and batch for the whole scene.
    // Synchronous in-process decisions block wall time, never advance physics.
    [DefaultExecutionOrder(-200)]
    public class JointControllerDriver : MonoBehaviour, ISceneMotionSink
    {
        readonly Dictionary<string, JointAgent> agents = new Dictionary<string, JointAgent>(StringComparer.Ordinal);
        SceneController core;
        Rigidbody robot;
        float robotRadius;
        double decisionPeriod, nextDecision;
        bool populationDirty, faulted;
        global::SEAN.Tasks.Base task;
        public SceneSnapshot LastSnapshot { get; private set; }
        public int DecisionCount { get; private set; }
        public bool Faulted { get { return faulted; } }

        public static JointControllerDriver ForScene()
        {
            var existing = FindObjectsOfType<JointControllerDriver>();
            if (existing.Length > 1) throw new InvalidOperationException("Multiple joint controller owners");
            if (existing.Length == 1) return existing[0];
            var driver = new GameObject("Joint pedestrian controller").AddComponent<JointControllerDriver>();
            try
            {
                if (SEAN.instance == null) throw new InvalidOperationException("Configure a driver before creating joint test avatars");
                var body = SEAN.instance.robot.base_link.GetComponent<Rigidbody>();
                var shapes = RobotAvoidance.BodyColliders(body);
                if (shapes.Length == 0) throw new InvalidOperationException("Robot has no physical footprint");
                // Conservative bound from owned physical shapes, not the old
                // 0.2-m placeholder. Clearance logging still uses full shapes.
                float radius = shapes.Max(c => {
                    var b = c.bounds; var offset = b.center - body.position;
                    return new Vector2(offset.x, offset.z).magnitude + new Vector2(b.extents.x, b.extents.z).magnitude;
                });
                driver.Initialize(new GoalTestController(.4), body, radius, .4);
                return driver;
            }
            catch { Destroy(driver.gameObject); throw; }
        }

        public void Initialize(ISceneController worker, Rigidbody externalRobot, float radius, double period)
        {
            if (core != null || worker == null || externalRobot == null || !Contract.Finite(radius) || radius <= 0 ||
                !Contract.Finite(period) || period < Time.fixedDeltaTime ||
                Math.Abs(period / Time.fixedDeltaTime - Math.Round(period / Time.fixedDeltaTime)) > 1e-4)
                throw new ArgumentException("One owner, robot footprint and whole-physics-step cadence required");
            robot = externalRobot; robotRadius = radius; decisionPeriod = period;
            workerDecision = worker.Decide;
            core = new SceneController(worker, this, period);
            if (SEAN.instance != null)
            {
                task = SEAN.instance.robotTask;
                task.onNewTask += ResetEpisode;
            }
            ResetEpisode();
        }

        public void Register(JointAgent agent)
        {
            if (core == null || agent == null || !agent.Initialized ||
                string.IsNullOrWhiteSpace(agent.StableId) || agent.StableId == "robot" || agents.ContainsKey(agent.StableId))
                throw new InvalidOperationException("Invalid or duplicate joint actor registration");
            agents.Add(agent.StableId, agent); PopulationChanged();
        }
        public void Unregister(JointAgent agent)
        {
            JointAgent registered;
            if (agent.StableId != null && agents.TryGetValue(agent.StableId, out registered) && registered == agent)
            {
                agents.Remove(agent.StableId); PopulationChanged();
            }
        }
        public void PopulationChanged()
        {
            Stop(); populationDirty = true;
        }
        public void ResetEpisode()
        {
            if (core == null) return;
            core.Reset(Guid.NewGuid().ToString("N"));
            LastSnapshot = null; nextDecision = double.NegativeInfinity;
            populationDirty = faulted = false;
        }

        public void StepBoundary(double simulationTime, float step)
        {
            if (core == null || faulted) return;
            try
            {
                if (populationDirty) ResetEpisode();
                core.Tick(simulationTime);
                if (simulationTime + 1e-7 >= nextDecision)
                {
                    var states = agents.Values.Where(a => a != null).Select(a => a.CaptureState()).ToList();
                    if (robot == null) throw new InvalidOperationException("External robot was removed");
                    states.Add(new ActorState("robot", true, true, Contract.FromUnity(robot.position),
                        Contract.FromUnity(robot.velocity), Contract.HeadingFromUnity(robot.rotation.eulerAngles.y * Mathf.Deg2Rad),
                        robotRadius, float.MaxValue, null));
                    // Geometry transport is not supplied by this open-floor
                    // test double. A geometry-aware adapter must provide it.
                    LastSnapshot = core.BeginStep(simulationTime, states, new Obstacle[0]);
                    if (!core.Apply(workerDecision(LastSnapshot)))
                        throw new InvalidOperationException("Invalid joint controller output");
                    nextDecision = simulationTime + decisionPeriod;
                    DecisionCount++;
                }
                foreach (var agent in agents.Values) if (agent != null) agent.ApplyAtBoundary(step);
            }
            catch
            {
                faulted = true; core.Fail(); throw;
            }
        }

        Func<SceneSnapshot, SceneDecision> workerDecision;
        void FixedUpdate()
        {
            if (core == null || faulted) return;
            try { StepBoundary(Time.fixedTimeAsDouble, Time.fixedDeltaTime); }
            catch (Exception error) { Debug.LogException(error, this); }
        }
        public void Commit(IReadOnlyList<MotionTarget> targets)
        {
            // Core already validated the complete batch. No callbacks or
            // physics advancement occur between these target assignments.
            foreach (var target in targets) agents[target.ActorId].SetTarget(target.Velocity);
        }
        public void Stop()
        {
            foreach (var agent in agents.Values) if (agent != null) agent.StopMotion();
        }
        void OnDisable()
        {
            Stop();
            if (core != null) { faulted = true; core.Fail(); }
        }
        void OnDestroy()
        {
            if (task != null) task.onNewTask -= ResetEpisode;
            if (core != null) core.Dispose();
        }
    }
}
