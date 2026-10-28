using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

namespace SEAN.Scenario.Agents
{
    // Explicit validation mode: uses the existing Warehouse floor, obstacles and baked NavMesh.
    public class WarehouseEncounterValidation : MonoBehaviour
    {
        [Serializable] public class Person
        {
            public string avatar;
            public Vector3 start, goal;
            public float radius, reachedSeconds = -1;
            public Vector3[] plan;
        }
        [Serializable] public class Sample
        {
            public float time;
            public Vector3 robot, robotVelocity;
            public Vector3[] people;
            public float[] robotClearances, personClearances, goalDistances;
            public bool yielding;
        }
        [Serializable] public class Episode
        {
            public int index, seed, mirror;
            public string layout, mode, error;
            public bool setup, complete;
            public Vector3 robotStart, robotGoal;
            public Vector3[] robotPlan;
            public float duration;
            public int robotPersonContacts, personPersonContacts, personObjectContacts, robotObjectContacts;
            public List<Person> people = new List<Person>();
            public List<Sample> samples = new List<Sample>();
        }
        [Serializable] public class Report
        {
            public string fixture = "Warehouse baked NavMesh and physical geometry; four animated SFAgents";
            public string model = "ColliderV1", contactModel = "VelocityBodyV1";
            public int schema = 1, layoutVersion = 3;
            public float step, deadline = 45, desiredSpeed = Parameters.DESIRED_SPEED;
            public int[] seeds;
            public string[] layouts, modes;
            public bool complete;
            public string error;
            public List<Episode> episodes = new List<Episode>();
        }
        [Serializable] public class GeometryProbe
        {
            public Vector3 requested, nearest;
            public bool found;
            public float distance;
        }
        [Serializable] public class GeometryAudit
        {
            public Vector3[] vertices;
            public int[] indices;
            public List<GeometryProbe> probes = new List<GeometryProbe>();
        }
        [Serializable] public class Request
        {
            public int index, expectedPeople;
            public Vector3 robotStart, robotGoal;
            public bool complete;
        }
        private Rigidbody body;
        private Collider[] shapes;
        private IVI.SFAgent[] agents = new IVI.SFAgent[0];
        private Episode current;
        private string mode, output, exchange;
        private bool running, yielding;
        private int waypoint;
        private global::SEAN.Control.VelocityController[] controllers;
        private WarehouseEncounterPopulation population;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (!System.Environment.GetCommandLineArgs().Contains("-warehouse-encounter-output")) return;
            if (FindObjectOfType<WarehouseEncounterValidation>() != null) return;
            var go = new GameObject("Warehouse encounter validation");
            DontDestroyOnLoad(go);
            go.AddComponent<WarehouseEncounterValidation>();
        }
        private string Argument(string name, string fallback)
        {
            string[] args = System.Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, name);
            return i < 0 ? fallback : args[i+1];
        }
        private IEnumerator Start()
        {
            while (SEAN.instance == null || SEAN.instance.robot == null) yield return null;
            yield return new WaitForSeconds(8);
            output = Argument("-warehouse-encounter-output", null);
            exchange = output + ".exchange";
            Directory.CreateDirectory(exchange);
            var report = new Report {
                step = float.Parse(Argument("-warehouse-encounter-step", "0.02"), CultureInfo.InvariantCulture),
                seeds = Argument("-warehouse-encounter-seeds", "7,42").Split(',').Select(int.Parse).ToArray(),
                layouts = Argument("-warehouse-encounter-layouts", "aisle,rack_end").Split(','),
                modes = Argument("-warehouse-encounter-modes", "absent,parked,yielding,oblivious,nav2").Split(',') };
            if (report.step != .02f && report.step != .04f) throw new ArgumentException("Unsupported step");
            if (report.seeds.Length == 0 || report.seeds.Distinct().Count() != report.seeds.Length
                || report.layouts.Length == 0 || report.layouts.Distinct().Count() != report.layouts.Length
                || report.layouts.Any(x => x != "aisle" && x != "rack_end")
                || report.modes.Length == 0 || report.modes.Distinct().Count() != report.modes.Length
                || report.modes.Any(x => !new[] { "absent", "parked", "yielding", "oblivious", "nav2" }.Contains(x)))
                throw new ArgumentException("Invalid encounter matrix");
            Time.captureDeltaTime = Time.fixedDeltaTime = report.step;
            Application.targetFrameRate = Mathf.RoundToInt(1/report.step);
            QualitySettings.vSyncCount = 0;
            SEAN.instance.robotTask.enabled = false;
            controllers = FindObjectsOfType<global::SEAN.Control.VelocityController>();
            foreach (var controller in controllers) controller.enabled = false;
            var behavior = SEAN.instance.pedestrianBehavior;
            var go = new GameObject("WarehouseEncounters");
            go.transform.SetParent(behavior.transform.parent, false);
            population = go.AddComponent<WarehouseEncounterPopulation>();
            SEAN.instance.SetPedestrianBehavior(go.name);
            body = SEAN.instance.robot.base_link.GetComponent<Rigidbody>();
            shapes = RobotAvoidance.BodyColliders(body);
            if (shapes.Length == 0 || shapes.Any(c => !(c is CapsuleCollider) && !(c is WheelCollider))
                || shapes.OfType<CapsuleCollider>().Any(c => c.direction != 1))
                throw new InvalidOperationException("Independent metric requires audited Kuri shapes");
            var mesh = NavMesh.CalculateTriangulation();
            var audit = new GeometryAudit { vertices = mesh.vertices, indices = mesh.indices };
            foreach (var point in new[] { new Vector3(1,0,-15), new Vector3(-5,0,5), new Vector3(-5,0,-8), Vector3.zero })
            {
                bool found = NavMesh.SamplePosition(point,out var hit,10,NavMesh.AllAreas);
                audit.probes.Add(new GeometryProbe { requested = point, nearest = hit.position, found = found, distance = hit.distance });
            }
            Save(output+".geometry.json",audit);
            var robotProbe = body.gameObject.AddComponent<WarehouseContactProbe>();
            robotProbe.robot = body;
            var template = Resources.Load<GameObject>("Prefabs/RocketboxRandomAnimatedAgent").GetComponent<RandomAvatar>();
            // Mode outermost: paired controls have exactly the same seed/avatar assignments.
            foreach (string kind in report.modes)
            foreach (int seed in report.seeds)
            foreach (string layout in report.layouts)
            foreach (int mirror in new[] { -1, 1 })
            {
                current = new Episode { index = report.episodes.Count, seed = seed, layout = layout, mirror = mirror, mode = kind };
                report.episodes.Add(current);
                mode = kind; running = false; yielding = false; waypoint = 1;
                foreach (var controller in controllers) controller.enabled = false;
                FreezeRobot();
                // ROS coordinates: x forward (Unity z), y left (-Unity x).
                Vector3 center = layout == "aisle" ? new Vector3(-5,0,5) : new Vector3(-5,0,-8);
                Func<float,float,Vector3> point = (lateral, longitudinal) => center + new Vector3(lateral*mirror,0,longitudinal);
                Vector3 start = layout == "aisle" ? point(0,-4) : point(-4,0);
                Vector3 goal = layout == "aisle" ? point(0,4) : point(4,0);
                if (kind == "parked") start = center;
                if (kind == "absent") start = new Vector3(1,0,-15);
                Vector3[] starts = new Vector3[4], goals = new Vector3[4];
                for (int i = 0; i < 4; i++)
                {
                    float lane = i%2 == 0 ? -.65f : .65f;
                    float depth = i < 2 ? 3.5f : 5f;
                    starts[i] = point(lane,depth);
                    // Preserve travel ordering at the terminal goals: leading people finish
                    // farther downstream, instead of parking across the trailing route.
                    float goalDepth = i < 2 ? 5f : 3.5f;
                    goals[i] = layout == "aisle" ? point(lane,-goalDepth) : point(4+lane,-goalDepth);
                }
                string setupError = null;
                try
                {
                    // No retries/replacement: a rejected fixed pose is a failed episode.
                    start = Surface(start); goal = Surface(goal);
                    current.robotStart = start + Vector3.up*.05f; current.robotGoal = goal;
                    current.robotPlan = Plan(start,goal);
                    if (!global::SEAN.Tasks.RandomTaskPlacement.IsClear(start,SEAN.instance.robot.gameObject.transform)
                        || !global::SEAN.Tasks.RandomTaskPlacement.IsClear(goal,SEAN.instance.robot.gameObject.transform))
                        throw new InvalidOperationException("Robot start/goal lacks map/physical clearance");
                    for (int i = 0; i < 4; i++)
                    {
                        starts[i] = Surface(starts[i]); goals[i] = Surface(goals[i]);
                        if (!PersonClear(starts[i]) || !PersonClear(goals[i])) throw new InvalidOperationException("Person start/goal blocked: " + i);
                        var avatar = template.avatars[(int)((uint)seed+(uint)i)%template.avatars.Length];
                        current.people.Add(new Person { avatar = avatar.name, start = starts[i], goal = goals[i], plan = Plan(starts[i],goals[i]) });
                    }
                    SetRobot(current.robotStart,Quaternion.LookRotation((goal-start).normalized == Vector3.zero ? Vector3.forward : (goal-start).normalized));
                    UnityEngine.Random.InitState(seed);
                    agents = new IVI.SFAgent[4];
                    for (int i = 0; i < 4; i++)
                    {
                        var avatar = Instantiate(template.avatars[(int)((uint)seed+(uint)i)%template.avatars.Length], starts[i],Quaternion.LookRotation(goals[i]-starts[i]));
                        avatar.name = "Encounter person " + i;
                        avatar.GetComponent<Animator>().runtimeAnimatorController = template.animationController;
                        agents[i] = avatar.AddComponent<IVI.SFAgent>();
                        agents[i].robotResponse = RobotResponse.ColliderV1;
                        agents[i].contactModel = IVI.PedestrianContactModel.VelocityBodyV1;
                    }
                    population.people = agents.Cast<Trajectory.TrackedAgent>().ToArray();
                }
                catch (Exception error) { setupError = error.ToString(); }
                if (setupError != null)
                {
                    current.error = setupError; Save(output,report);
                    Debug.LogError("Warehouse encounter setup rejected: " + setupError);
                    CleanupPeople();
                    continue;
                }
                yield return null;
                var probes = new WarehouseContactProbe[4];
                for (int i = 0; i < 4; i++)
                {
                    current.people[i].radius = agents[i].GetComponent<CapsuleCollider>().radius;
                    probes[i] = agents[i].gameObject.AddComponent<WarehouseContactProbe>(); probes[i].robot = body;
                    agents[i].InitDest(goals[i]);
                    agents[i].enabled = false;
                    agents[i].GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeAll;
                    agents[i].StopAnimator();
                }
                current.setup = true;
                Save(output,report);
                if (kind == "nav2")
                {
                    Save(Path.Combine(exchange,"request.json"),new Request { index = current.index, robotStart = current.robotStart, robotGoal = goal, expectedPeople = 4 });
                    Save(output,report);
                    float until = Time.realtimeSinceStartup + 120;
                    while (!File.Exists(Path.Combine(exchange,"go-"+current.index)) && Time.realtimeSinceStartup < until) yield return null;
                    if (!File.Exists(Path.Combine(exchange,"go-"+current.index)))
                    {
                        current.error = "Nav2 handshake wall timeout"; Save(output,report); CleanupPeople(); continue;
                    }
                }
                SetRobot(current.robotStart,Quaternion.LookRotation((goal-start).normalized == Vector3.zero ? Vector3.forward : (goal-start).normalized));
                body.constraints = RigidbodyConstraints.FreezeRotation | RigidbodyConstraints.FreezePositionY;
                if (kind == "nav2") foreach (var controller in controllers) controller.enabled = true;
                for (int i = 0; i < 4; i++)
                {
                    agents[i].GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeRotation;
                    agents[i].enabled = true;
                    agents[i].InitDest(goals[i]);
                }
                int robotPersonBaseline = robotProbe.person, robotObjectBaseline = robotProbe.objects;
                float started = Time.time;
                current.samples.Add(Capture(0));
                running = true;
                for (int frame = 0; frame < Mathf.RoundToInt(report.deadline/report.step); frame++)
                {
                    yield return null;
                    float elapsed = Time.time-started;
                    var sample = Capture(elapsed); current.samples.Add(sample);
                    for (int i = 0; i < 4; i++)
                        if (current.people[i].reachedSeconds < 0 && sample.goalDistances[i] <= 1) current.people[i].reachedSeconds = elapsed;
                    bool robotDone = kind == "parked" || kind == "absent" || Horizontal(body.position,goal) <= .4f;
                    if (kind == "nav2") robotDone &= File.Exists(Path.Combine(exchange,"done-"+current.index));
                    if (current.people.All(p => p.reachedSeconds >= 0) && robotDone) break;
                }
                running = false;
                foreach (var controller in controllers) controller.enabled = false;
                body.velocity = body.angularVelocity = Vector3.zero;
                current.duration = Time.time-started;
                current.robotPersonContacts = robotProbe.person-robotPersonBaseline;
                current.robotObjectContacts = robotProbe.objects-robotObjectBaseline;
                // Each person/person callback is recorded on both bodies; retain the raw double count.
                current.personPersonContacts = probes.Sum(p => p.person);
                current.personObjectContacts = probes.Sum(p => p.objects);
                current.complete = true;
                if (kind == "nav2") Save(Path.Combine(exchange,"ended-"+current.index),new Request { index = current.index });
                Debug.Log("Warehouse encounter: " + kind + " " + layout + " mirror=" + mirror + " seed=" + seed
                    + " duration=" + current.duration + " peopleReached=" + current.people.Count(p => p.reachedSeconds >= 0)
                    + " robotContacts=" + current.robotPersonContacts + " personContacts=" + current.personPersonContacts + " wallContacts=" + current.personObjectContacts);
                CleanupPeople(); yield return null;
                Save(output,report);
            }
            report.complete = true; Save(output,report);
            Save(Path.Combine(exchange,"request.json"),new Request { complete = true });
            Debug.Log("Warehouse encounters complete: " + output);
        }
        private void FreezeRobot()
        {
            foreach (var rb in SEAN.instance.robot.GetComponentsInChildren<Rigidbody>())
            {
                rb.velocity = rb.angularVelocity = Vector3.zero;
                rb.useGravity = false; rb.constraints = RigidbodyConstraints.FreezeAll;
            }
        }
        private void SetRobot(Vector3 position, Quaternion rotation)
        {
            body.transform.SetPositionAndRotation(position,rotation);
            body.position = position; body.rotation = rotation;
            body.velocity = body.angularVelocity = Vector3.zero;
            Physics.SyncTransforms();
        }
        private static Vector3 Surface(Vector3 point)
        {
            if (!NavMesh.SamplePosition(point,out var hit,.2f,NavMesh.AllAreas) || Vector3.Distance(point,hit.position) > .2f)
                throw new InvalidOperationException("Fixed pose not on baked Warehouse NavMesh: " + point);
            return hit.position;
        }
        private bool PersonClear(Vector3 point)
        {
            return !Physics.OverlapCapsule(point+Vector3.up*.3f,point+Vector3.up*1.5f,.21f,~0,QueryTriggerInteraction.Ignore)
                .Any(c => !c.transform.IsChildOf(SEAN.instance.robot.gameObject.transform));
        }
        private static Vector3[] Plan(Vector3 start, Vector3 goal)
        {
            var path = new NavMeshPath();
            if (!NavMesh.CalculatePath(start,goal,NavMesh.AllAreas,path) || path.status != NavMeshPathStatus.PathComplete || path.corners.Length < 2)
                throw new InvalidOperationException("Fixed route is not connected on baked NavMesh");
            return path.corners;
        }
        private void CleanupPeople()
        {
            foreach (var agent in agents)
                if (agent != null) { IVI.SFAgent.GO2Agent.Remove(agent.gameObject); Destroy(agent.gameObject); }
            agents = new IVI.SFAgent[0]; population.people = new Trajectory.TrackedAgent[0];
        }
        private static float Horizontal(Vector3 a, Vector3 b) { a.y = b.y = 0; return Vector3.Distance(a,b); }
        private Sample Capture(float time)
        {
            var positions = agents.Select(a => a.transform.position).ToArray();
            var gaps = new List<float>();
            for (int i = 0; i < agents.Length; i++) for (int j = i+1; j < agents.Length; j++)
                gaps.Add(Horizontal(positions[i],positions[j])-current.people[i].radius-current.people[j].radius);
            return new Sample { time = time, robot = body.position, robotVelocity = body.velocity, people = positions,
                robotClearances = positions.Select((p,i) => shapes.Min(c => RobotClearance(c,p))-current.people[i].radius).ToArray(),
                personClearances = gaps.ToArray(), goalDistances = positions.Select((p,i) => Horizontal(p,current.people[i].goal)).ToArray(), yielding = yielding };
        }
        private void FixedUpdate()
        {
            if (!running || body == null || mode == "nav2") return;
            if (mode == "parked" || mode == "absent") { body.velocity = Vector3.zero; return; }
            Vector3 goal = current.robotPlan[Mathf.Min(waypoint,current.robotPlan.Length-1)];
            if (Horizontal(body.position,goal) <= .15f && waypoint < current.robotPlan.Length-1) goal = current.robotPlan[++waypoint];
            Vector3 direction = goal-body.position; direction.y = 0;
            yielding = mode == "yielding" && agents.Any(p => {
                Vector3 delta = p.transform.position-body.position; delta.y = 0;
                return delta.magnitude < 2 && Vector3.Dot(delta,direction.normalized) > -.2f;
            });
            body.rotation = Quaternion.LookRotation(direction.sqrMagnitude > .0001f ? direction : body.transform.forward);
            body.velocity = yielding || Horizontal(body.position,current.robotGoal) <= .3f ? Vector3.zero : direction.normalized*.3f;
        }
        // Independent x/z projections of the audited actual Kuri shapes; no force helper queries.
        private static float RobotClearance(Collider c, Vector3 point)
        {
            Vector3 center, scale = c.transform.lossyScale; float radius;
            if (c is CapsuleCollider capsule) { center = c.transform.TransformPoint(capsule.center); radius = capsule.radius*Mathf.Max(Mathf.Abs(scale.x),Mathf.Abs(scale.z)); }
            else { var wheel = (WheelCollider)c; center = c.transform.TransformPoint(wheel.center); radius = wheel.radius*Mathf.Max(Mathf.Abs(scale.x),Mathf.Abs(scale.y),Mathf.Abs(scale.z)); }
            return Horizontal(point,center)-radius;
        }
        private static void Save<T>(string path, T data)
        {
            File.WriteAllText(path+".tmp",JsonUtility.ToJson(data,true));
            if (File.Exists(path)) File.Replace(path+".tmp",path,null); else File.Move(path+".tmp",path);
        }
    }
    public class WarehouseEncounterPopulation : PedestrianBehavior.Base
    {
        public Trajectory.TrackedAgent[] people = new Trajectory.TrackedAgent[0];
        public override Trajectory.TrackedAgent[] agents { get { return people; } }
        public override Trajectory.TrackedGroup[] groups { get { return new Trajectory.TrackedGroup[0]; } }
        public override string scenario_name { get { return "WarehouseEncounters"; } }
    }
    public class WarehouseContactProbe : MonoBehaviour
    {
        public Rigidbody robot;
        public int person, robotContacts, objects;
        private void OnCollisionEnter(Collision collision)
        {
            if (collision.collider.GetComponentInParent<IVI.SFAgent>() != null) { person++; return; }
            if (collision.rigidbody == robot) { robotContacts++; return; }
            for (int i = 0; i < collision.contactCount; i++)
                if (collision.GetContact(i).normal.y < .5f) { objects++; return; }
        }
    }
}
