using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.TestTools;
using SEAN.Scenario.Controllers;

public class JointAvatarTests
{
    static Type FindType(string name) { return AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType(name)).First(t => t != null); }
    static object Invoke(object instance, string name, params object[] args)
    {
        try
        {
            var method = instance.GetType().GetMethods().Single(m => m.Name == name &&
                m.GetParameters().Length == args.Length &&
                m.GetParameters().Select((p, i) => p.ParameterType.IsInstanceOfType(args[i])).All(x => x));
            return method.Invoke(instance, args);
        }
        catch (TargetInvocationException error) { throw error.InnerException; }
    }
    sealed class RecordingWorker : ISceneController
    {
        readonly ISceneController goal;
        readonly System.Diagnostics.Process process;
        bool disposed;
        [Serializable] sealed class Startup { public int port; }
        public RecordingWorker(bool socket)
        {
            if (!socket) { goal = new GoalTestController(.4); return; }
            var start = new System.Diagnostics.ProcessStartInfo("python3", "-u \"" +
                Environment.GetEnvironmentVariable("SEAN_WORKER_SCRIPT") +
                "\" --test-double --moving-test-double --port 0 --observation-steps 3 --dt .4") {
                UseShellExecute = false, RedirectStandardOutput = true, RedirectStandardError = true
            };
            process = System.Diagnostics.Process.Start(start);
            try
            {
                var line = process.StandardOutput.ReadLineAsync();
                if (!line.Wait(5000) || line.Result == null) throw new Exception("Animated worker startup failed");
                goal = new SocketSceneController(JsonUtility.FromJson<Startup>(line.Result).port, 3, .4);
            }
            catch { if (!process.HasExited) process.Kill(); process.Dispose(); throw; }
        }
        public readonly List<SceneSnapshot> Snapshots = new List<SceneSnapshot>();
        public bool Broken;
        public int Resets;
        public void Reset(string id) { Resets++; Snapshots.Clear(); goal.Reset(id); }
        public void Dispose()
        {
            if (disposed) return;
            disposed = true; goal.Dispose();
            if (process != null)
            {
                if (!process.WaitForExit(3000)) { process.Kill(); process.WaitForExit(); }
                process.Dispose();
            }
        }
        public SceneDecision Decide(SceneSnapshot snapshot)
        {
            if (Broken) throw new InvalidOperationException("test worker lost");
            Snapshots.Add(snapshot); return goal.Decide(snapshot);
        }
    }

    [UnityTest]
    public IEnumerator AnimatedAvatarsUseJointPhysicsTargetsAndStopOnResetAndWorkerLoss()
    {
        var exercise = ExerciseAvatars(false);
        try { while (exercise.MoveNext()) yield return exercise.Current; }
        finally { (exercise as IDisposable)?.Dispose(); }
    }
    [UnityTest]
    public IEnumerator AnimatedAvatarsUsePythonWorkerHistoryAndPhysicalTargets()
    {
        if (string.IsNullOrEmpty(Environment.GetEnvironmentVariable("SEAN_WORKER_SCRIPT")))
            Assert.Ignore("Set SEAN_WORKER_SCRIPT for animated cross-process validation");
        var exercise = ExerciseAvatars(true);
        try { while (exercise.MoveNext()) yield return exercise.Current; }
        finally { (exercise as IDisposable)?.Dispose(); }
    }
    IEnumerator ExerciseAvatars(bool socket)
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        yield return new EnterPlayMode();
        float oldCapture = Time.captureDeltaTime, oldFixed = Time.fixedDeltaTime;
        int oldRate = Application.targetFrameRate;
        var objects = new List<GameObject>();
        RecordingWorker worker = null;
        NavMeshDataInstance mesh = default;
        try
        {
            Time.captureDeltaTime = Time.fixedDeltaTime = .02f;
            Application.targetFrameRate = -1;
            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube); objects.Add(floor);
            floor.transform.position = Vector3.down * .1f;
            floor.transform.localScale = new Vector3(20, .2f, 20);
            var source = new NavMeshBuildSource { shape = NavMeshBuildSourceShape.Box,
                transform = Matrix4x4.TRS(floor.transform.position, Quaternion.identity, Vector3.one),
                size = floor.transform.localScale, area = 0 };
            var data = NavMeshBuilder.BuildNavMeshData(NavMesh.GetSettingsByIndex(0),
                new List<NavMeshBuildSource> { source }, new Bounds(Vector3.zero, new Vector3(22, 5, 22)),
                Vector3.zero, Quaternion.identity);
            Assert.NotNull(data); mesh = NavMesh.AddNavMeshData(data);
            var robot = new GameObject("external robot"); objects.Add(robot);
            robot.transform.position = new Vector3(8, 0, 8);
            var robotBody = robot.AddComponent<Rigidbody>(); robotBody.isKinematic = true;
            var driverObject = new GameObject("joint owner"); objects.Add(driverObject);
            var driver = (MonoBehaviour)driverObject.AddComponent(FindType("SEAN.Scenario.Agents.JointControllerDriver"));
            worker = new RecordingWorker(socket);
            Invoke(driver, "Initialize", worker, robotBody, .5f, .4);
            var template = Resources.Load<GameObject>("Prefabs/RocketboxRandomAnimatedAgent")
                .GetComponent(FindType("SEAN.Scenario.Agents.RandomAvatar"));
            var choices = (GameObject[])template.GetType().GetField("avatars").GetValue(template);
            var animation = (RuntimeAnimatorController)template.GetType().GetField("animationController").GetValue(template);
            var avatars = new GameObject[2]; var agents = new MonoBehaviour[2];
            var starts = new[] { new Vector3(-2, 0, -3), new Vector3(2, 0, 3) };
            var goals = new[] { new Vector3(-2, 0, 3), new Vector3(2, 0, -3) };
            for (int i = 0; i < 2; i++)
            {
                avatars[i] = UnityEngine.Object.Instantiate(choices[7+i], starts[i], Quaternion.LookRotation(goals[i]-starts[i]));
                objects.Add(avatars[i]); avatars[i].name = "person-" + i;
                avatars[i].GetComponent<Animator>().runtimeAnimatorController = animation;
                agents[i] = (MonoBehaviour)avatars[i].AddComponent(FindType("SEAN.Scenario.Agents.JointAgent"));
            }
            yield return null;
            for (int i = 0; i < 2; i++)
            {
                Assert.That(avatars[i].GetComponent<Animator>().updateMode, Is.EqualTo(AnimatorUpdateMode.AnimatePhysics));
                Assert.False(avatars[i].GetComponent<Animator>().applyRootMotion,
                    "Animation must not add translation to the physical velocity target");
                Invoke(agents[i], "InitDest", goals[i]);
            }
            float until = Time.time + 3;
            while (Time.time < until)
            {
                yield return null;
                foreach (var avatar in avatars)
                {
                    var achieved = avatar.GetComponent<Rigidbody>().velocity;
                    achieved.y = 0;
                    Assert.LessOrEqual(achieved.magnitude,.61f,"Root animation added uncommanded translation");
                }
            }
            Assert.Greater(worker.Snapshots.Count, 3, "Joint scene decisions never ran");
            foreach (var snapshot in worker.Snapshots) Assert.AreEqual(3, snapshot.Actors.Count);
            for (int i = 0; i < 2; i++)
            {
                Vector3 displacement = avatars[i].transform.position - starts[i]; displacement.y = 0;
                Assert.Greater(Vector3.Dot(displacement, (goals[i]-starts[i]).normalized), .3f,
                    "Actual production body did not follow the joint target");
                Assert.Less(displacement.magnitude, 2.5f, "Possible double integration");
                var state = worker.Snapshots.Last().Actors.Single(a => a.Id == "person-" + i);
                Assert.Greater((state.Position-Contract.FromUnity(starts[i])).magnitude, .2f,
                    "Worker received predicted state instead of actual executed motion");
            }
            // Reset clears targets and physical velocity synchronously, before another frame.
            string oldEpisode = worker.Snapshots.Last().EpisodeId;
            Invoke(driver, "ResetEpisode");
            for (int i = 0; i < 2; i++)
            {
                Assert.AreEqual(Vector3.zero, avatars[i].GetComponent<Rigidbody>().velocity);
                Assert.AreEqual(Vector3.zero, agents[i].GetType().GetProperty("TargetVelocity").GetValue(agents[i]));
            }
            yield return null;
            Assert.AreNotEqual(oldEpisode, worker.Snapshots.Last().EpisodeId);
            if (socket)
            {
                until = Time.time + 1.2f;
                while (Time.time < until) yield return null;
            }
            // Inject worker loss at a synchronous boundary. The normal driver
            // remains stopped until explicit reset; the robot is untouched.
            worker.Broken = true;
            var robotPosition = robotBody.position;
            Assert.Greater(((Vector3)agents[0].GetType().GetProperty("TargetVelocity").GetValue(agents[0])).magnitude,.1f,
                "Worker-loss check must begin while the avatar has a nonzero target");
            Assert.Throws<InvalidOperationException>(() => Invoke(driver, "StepBoundary", Time.fixedTimeAsDouble + .4, .02f));
            var stopped = avatars.Select(a => a.transform.position).ToArray();
            until = Time.time + .5f;
            while (Time.time < until) yield return null;
            for (int i = 0; i < 2; i++)
            {
                var difference = avatars[i].transform.position - stopped[i]; difference.y = 0;
                Assert.Less(difference.magnitude, .005f, "Worker loss left residual root motion");
            }
            Assert.AreEqual(robotPosition, robotBody.position);
            Assert.True((bool)driver.GetType().GetProperty("Faulted").GetValue(driver));
        }
        finally
        {
            foreach (var go in objects.AsEnumerable().Reverse()) if (go != null) UnityEngine.Object.Destroy(go);
            if (worker != null) worker.Dispose();
            if (mesh.valid) mesh.Remove();
            Time.captureDeltaTime = oldCapture; Time.fixedDeltaTime = oldFixed;
            Application.targetFrameRate = oldRate;
        }
    }
    [UnityTearDown] public IEnumerator LeavePlayMode()
    {
        if (Application.isPlaying) yield return new ExitPlayMode();
    }
}
