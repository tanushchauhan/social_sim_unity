using System;
using System.Collections.Generic;
using System.Linq;
using System.Globalization;
using UnityEngine;
using UnityEngine.AI;
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
        IReadOnlyList<Obstacle> obstacles = new Obstacle[0];
        public string GeometryModel { get; private set; } = "open_floor_test";
        public string ControllerName { get; private set; }
        public OrcaSettings OrcaConfiguration { get; private set; }
        public float RobotRadius { get { return robotRadius; } }
        public int ObstacleCount { get { return obstacles.Count; } }
        public string EpisodeId { get { return core == null ? null : core.EpisodeId; } }
        public readonly List<double> InferenceMilliseconds = new List<double>();
        public SceneSnapshot LastSnapshot { get; private set; }
        public int DecisionCount { get; private set; }
        public bool Faulted { get { return faulted; } }

        public static JointControllerDriver ForScene(bool useOrca)
        {
            var existing = FindObjectsOfType<JointControllerDriver>();
            if (existing.Length > 1) throw new InvalidOperationException("Multiple joint controller owners");
            if (existing.Length == 1)
            {
                if ((useOrca && existing[0].ControllerName != "orca") ||
                    (!useOrca && existing[0].ControllerName == "orca"))
                    throw new InvalidOperationException("Cannot mix joint pedestrian controller owners");
                return existing[0];
            }
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
                if (useOrca)
                {
                    var settings = new OrcaSettings();
                    string[] args = System.Environment.GetCommandLineArgs();
                    for (int i = 0; i + 1 < args.Length; i++)
                    {
                        if (args[i] == "-orca-decision-period") settings.decisionPeriod = double.Parse(args[i+1],CultureInfo.InvariantCulture);
                        else if (args[i] == "-orca-neighbor-distance") settings.neighborDistance = float.Parse(args[i+1],CultureInfo.InvariantCulture);
                        else if (args[i] == "-orca-max-neighbors") settings.maxNeighbors = int.Parse(args[i+1],CultureInfo.InvariantCulture);
                        else if (args[i] == "-orca-agent-horizon") settings.agentHorizon = float.Parse(args[i+1],CultureInfo.InvariantCulture);
                        else if (args[i] == "-orca-obstacle-horizon") settings.obstacleHorizon = float.Parse(args[i+1],CultureInfo.InvariantCulture);
                    }
                    settings.Validate();
                    driver.obstacles = NavMeshControllerGeometry.Capture(body.position);
                    driver.GeometryModel = "NavMeshBoundaryV1";
                    driver.OrcaConfiguration = settings;
                    driver.Initialize(new OrcaSceneController(settings), body, radius, settings.decisionPeriod);
                    Debug.Log("ORCA controller initialized: " + JsonUtility.ToJson(settings) + " geometry=" + driver.GeometryModel + " polygons=" + driver.obstacles.Count);
                }
                else driver.Initialize(new GoalTestController(.4), body, radius, .4);
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
            ControllerName = worker is OrcaSceneController ? "orca" : worker is GoalTestController ? "joint_test" : "custom_test";
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
            {
                faulted = true;
                if (core != null) core.Fail();
                throw new InvalidOperationException("Invalid or duplicate joint actor registration");
            }
            if (GeometryModel == "NavMeshBoundaryV1")
            {
                var path = new NavMeshPath();
                if (!NavMesh.CalculatePath(agent.transform.position, robot.position, NavMesh.AllAreas, path) ||
                    path.status != NavMeshPathStatus.PathComplete)
                {
                    faulted = true; core.Fail();
                    throw new InvalidOperationException("ORCA actor is outside the robot's connected floor; entire scene stopped");
                }
            }
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
            InferenceMilliseconds.Clear(); DecisionCount = 0;
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
                    LastSnapshot = core.BeginStep(simulationTime, states, obstacles);
                    var timer = System.Diagnostics.Stopwatch.StartNew();
                    var decision = workerDecision(LastSnapshot);
                    timer.Stop(); InferenceMilliseconds.Add(timer.Elapsed.TotalMilliseconds);
                    if (!core.Apply(decision))
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
