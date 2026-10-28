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
    // Opt-in diagnostic. Normal scenarios and their simulation stepping are untouched.
    public class PedestrianAvoidanceValidation : MonoBehaviour
    {
        [Serializable] public class Sample
        {
            public float time, clearance, goalDistance;
            public Vector3 person, personVelocity, robot, robotVelocity;
        }
        [Serializable] public class Episode
        {
            public int seed, contacts;
            public string mode, avatar;
            public Vector3 expectedRobotStart;
            public float offset, heading, minimumClearance = float.MaxValue;
            public float reachedSeconds = -1;
            public bool reached;
            public List<Sample> samples = new List<Sample>();
        }
        [Serializable] public class Report
        {
            public string fixture = "open floor, production Kuri and animated SFAgent";
            public string model;
            public int[] seeds;
            public float step, desiredSpeed = Parameters.DESIRED_SPEED;
            public bool complete;
            public List<Episode> episodes = new List<Episode>();
        }
        private Rigidbody body;
        private IVI.SFAgent person;
        private string mode;
        private Vector3 origin = new Vector3(1000, 100, 1000);
        private bool yielded;
        private NavMeshDataInstance navMesh;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (!System.Environment.GetCommandLineArgs().Contains("-pedestrian-validation-output")) return;
            if (FindObjectOfType<PedestrianAvoidanceValidation>() != null) return;
            var go = new GameObject("Pedestrian avoidance validation");
            DontDestroyOnLoad(go);
            go.AddComponent<PedestrianAvoidanceValidation>();
        }

        private IEnumerator Start()
        {
            while (SEAN.instance == null || SEAN.instance.robot == null) yield return null;
            yield return new WaitForSeconds(8);
            string[] args = System.Environment.GetCommandLineArgs();
            string output = args[Array.IndexOf(args, "-pedestrian-validation-output") + 1];
            int index = Array.IndexOf(args, "-pedestrian-validation-step");
            float step = index < 0 ? .02f : float.Parse(args[index+1], CultureInfo.InvariantCulture);
            index = Array.IndexOf(args, "-pedestrian-validation-seeds");
            int[] seeds = index < 0 ? new[] { 7, 42 } : args[index+1].Split(',').Select(int.Parse).ToArray();
            if (step != .02f && step != .04f) throw new ArgumentException("Supported steps: .02 or .04");
            Time.captureDeltaTime = Time.fixedDeltaTime = step;
            Application.targetFrameRate = 100;
            QualitySettings.vSyncCount = 0;
            index = Array.IndexOf(args, "-pedestrian-validation-model");
            var model = index < 0 ? RobotResponse.ColliderV1 : (RobotResponse)Enum.Parse(typeof(RobotResponse),args[index+1]);
            var report = new Report { step = step, seeds = seeds, model = model.ToString() };
            var robot = SEAN.instance.robot;
            SEAN.instance.robotTask.enabled = false;
            foreach (var controller in FindObjectsOfType<global::SEAN.Control.VelocityController>()) controller.enabled = false;
            body = robot.base_link.GetComponent<Rigidbody>();
            foreach (var rb in robot.GetComponentsInChildren<Rigidbody>())
            {
                rb.velocity = rb.angularVelocity = Vector3.zero;
                rb.useGravity = false;
                rb.constraints = RigidbodyConstraints.FreezeAll;
            }
            var shapes = body.GetComponentsInChildren<Collider>().Where(c => !c.isTrigger && c.enabled && c.attachedRigidbody == body).ToArray();
            if (shapes.Length == 0 || shapes.Any(c => !(c is CapsuleCollider) && !(c is WheelCollider)))
                throw new InvalidOperationException("Independent metric requires audited Kuri capsules/wheels");
            if (shapes.OfType<CapsuleCollider>().Any(c => c.direction != 1))
                throw new InvalidOperationException("Metric requires vertical robot capsules");
            var probe = body.gameObject.AddComponent<PedestrianCollisionProbe>();
            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "Diagnostic floor";
            floor.transform.position = origin-Vector3.up*.1f;
            floor.transform.localScale = new Vector3(30,.2f,30);
            var source = new NavMeshBuildSource { shape = NavMeshBuildSourceShape.Box,
                transform = Matrix4x4.TRS(floor.transform.position, Quaternion.identity, Vector3.one),
                size = floor.transform.localScale, area = 0 };
            var data = NavMeshBuilder.BuildNavMeshData(NavMesh.GetSettingsByIndex(0),
                new List<NavMeshBuildSource> { source }, new Bounds(origin, new Vector3(32,5,32)), Vector3.zero, Quaternion.identity);
            if (data == null) throw new InvalidOperationException("Diagnostic NavMesh failed");
            navMesh = NavMesh.AddNavMeshData(data);
            var template = Resources.Load<GameObject>("Prefabs/RocketboxRandomAnimatedAgent").GetComponent<RandomAvatar>();
            foreach (int seed in seeds)
            foreach (string kind in new[] { "parked", "absent", "moving", "yielding" })
            foreach (float heading in kind == "parked" ? new[] { 0f, 180f } : new[] { 0f })
            foreach (float offset in kind == "parked" ? new[] { -.35f, 0f, .35f } : new[] { -.35f, .35f })
            {
                UnityEngine.Random.InitState(seed);
                body.constraints = kind == "moving" || kind == "yielding"
                    ? RigidbodyConstraints.FreezeRotation | RigidbodyConstraints.FreezePositionY : RigidbodyConstraints.FreezeAll;
                Vector3 robotStart = origin + new Vector3(kind == "absent" ? 30 : 0, .05f,
                    kind == "moving" || kind == "yielding" ? -1 : 0);
                body.transform.SetPositionAndRotation(robotStart, Quaternion.Euler(0, heading, 0));
                body.position = robotStart; body.rotation = body.transform.rotation;
                Physics.SyncTransforms();
                body.velocity = body.angularVelocity = Vector3.zero;
                mode = kind; yielded = false;
                var avatar = Instantiate(template.avatars[seed % template.avatars.Length], origin+new Vector3(offset,0,3), Quaternion.Euler(0,180,0));
                avatar.GetComponent<Animator>().runtimeAnimatorController = template.animationController;
                person = avatar.AddComponent<IVI.SFAgent>();
                person.robotResponse = model;
                // Start initializes the production animator, body, perception and planner.
                yield return null;
                var goal = origin + new Vector3(offset,0,-3);
                NavMeshHit hit;
                if (!NavMesh.SamplePosition(goal, out hit,.25f,NavMesh.AllAreas))
                    throw new InvalidOperationException("Diagnostic goal not on NavMesh");
                if (Vector3.Distance(body.position, robotStart) > .01f)
                    throw new InvalidOperationException("Diagnostic robot did not remain at requested start: " + body.position + " vs " + robotStart);
                person.InitDest(goal);
                var episode = new Episode { seed = seed, mode = kind, offset = offset, heading = heading, avatar = avatar.name, expectedRobotStart = robotStart };
                report.episodes.Add(episode);
                int baseline = probe.count;
                float started = Time.time;
                for (int frame = 0; frame < Mathf.RoundToInt(18/step); frame++)
                {
                    yield return null;
                    var capsule = person.GetComponent<CapsuleCollider>();
                    Vector3 p = capsule.transform.TransformPoint(capsule.center);
                    float clearance = shapes.Min(c => Clearance(c,p)) - capsule.radius*Mathf.Max(Mathf.Abs(capsule.transform.lossyScale.x),Mathf.Abs(capsule.transform.lossyScale.z));
                    float distance = Vector2.Distance(new Vector2(person.transform.position.x,person.transform.position.z),new Vector2(goal.x,goal.z));
                    episode.minimumClearance = Mathf.Min(episode.minimumClearance,clearance);
                    episode.samples.Add(new Sample { time = Time.time-started, clearance = clearance, goalDistance = distance,
                        person = person.transform.position, personVelocity = person.velocity, robot = body.position, robotVelocity = body.velocity });
                    if (distance <= 1f)
                    {
                        episode.reached = true; episode.reachedSeconds = Time.time-started; break;
                    }
                }
                episode.contacts = probe.count-baseline;
                person = null; mode = null; body.velocity = Vector3.zero;
                IVI.SFAgent.GO2Agent.Remove(avatar);
                Destroy(avatar);
                yield return null;
                Save(output, report);
                Debug.Log("Pedestrian diagnostic: " + kind + " seed=" + seed + " offset=" + offset + " heading=" + heading
                    + " minClearance=" + episode.minimumClearance + " reached=" + episode.reached + " contacts=" + episode.contacts);
            }
            report.complete = true;
            Save(output, report);
            Debug.Log("Pedestrian avoidance validation complete: " + output);
        }
        private static void Save(string output, Report report)
        {
            File.WriteAllText(output+".tmp", JsonUtility.ToJson(report, true));
            if (File.Exists(output)) File.Replace(output+".tmp",output,null);
            else File.Move(output+".tmp",output);
        }
        private void FixedUpdate()
        {
            if (body == null || person == null || (mode != "moving" && mode != "yielding")) return;
            Vector3 delta = person.transform.position-body.position; delta.y=0;
            if (mode == "yielding" && delta.magnitude <= 2) yielded = true;
            body.velocity = yielded ? Vector3.zero : Vector3.forward*.3f;
        }
        private static float Clearance(Collider collider, Vector3 p)
        {
            Vector3 center; float radius;
            if (collider is CapsuleCollider capsule)
            {
                center = capsule.transform.TransformPoint(capsule.center);
                radius = capsule.radius*Mathf.Max(Mathf.Abs(capsule.transform.lossyScale.x),Mathf.Abs(capsule.transform.lossyScale.z));
            }
            else
            {
                var wheel = (WheelCollider)collider;
                center = wheel.transform.TransformPoint(wheel.center);
                Vector3 s = wheel.transform.lossyScale;
                radius = wheel.radius*Mathf.Max(Mathf.Abs(s.x),Mathf.Abs(s.y),Mathf.Abs(s.z));
            }
            return Vector2.Distance(new Vector2(p.x,p.z),new Vector2(center.x,center.z))-radius;
        }
        private void OnDestroy() { if (navMesh.valid) navMesh.Remove(); }
    }
    public class PedestrianCollisionProbe : MonoBehaviour
    {
        public int count;
        private void OnCollisionEnter(Collision collision)
        {
            if (collision.collider.GetComponentInParent<IVI.SFAgent>() != null) count++;
        }
    }
}
