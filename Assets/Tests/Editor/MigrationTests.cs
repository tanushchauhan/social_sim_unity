using System;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

public class MigrationTests
{
    [Test]
    public void RocketboxImporterSkipsModelsWithoutItsSkeleton()
    {
        var model = new GameObject("unrelated model");
        var child = new GameObject("unrelated child");
        child.transform.SetParent(model.transform, false);
        try
        {
            Type importerType = FindType("FixRocketboxMaxImport");
            object importer = Activator.CreateInstance(importerType);
            MethodInfo postprocess = importerType.GetMethod("OnPostprocessModel",
                BindingFlags.NonPublic | BindingFlags.Instance);
            Assert.DoesNotThrow(() => postprocess.Invoke(importer, new object[] { model }));
            Assert.That(child.transform.parent, Is.EqualTo(model.transform));
            Assert.That(child.activeSelf, Is.True);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(model);
        }
    }

    [Test]
    public void DestroyedConnectionCancelsWorkerAndRemovesOnlyItsHeader()
    {
        Type connectionType = FindType("Unity.Robotics.ROSTCPConnector.ROSConnection");
        Type hudType = FindType("Unity.Robotics.ROSTCPConnector.HudPanel");
        var headers = (System.Collections.IDictionary)hudType
            .GetField("s_HeaderContents", BindingFlags.NonPublic | BindingFlags.Static).GetValue(null);
        var gameObject = new GameObject("connection cleanup");
        var cancellation = new System.Threading.CancellationTokenSource();
        Action otherHeader = () => { };
        try
        {
            Component connection = gameObject.AddComponent(connectionType);
            connectionType.GetField("m_ConnectionThreadCancellation", BindingFlags.NonPublic | BindingFlags.Instance)
                .SetValue(connection, cancellation);
            Action header = (Action)Delegate.CreateDelegate(typeof(Action), connection,
                connectionType.GetMethod("DrawHeaderGUI", BindingFlags.NonPublic | BindingFlags.Instance));
            hudType.GetMethod("RegisterHeader").Invoke(null, new object[] { otherHeader, 900 });
            hudType.GetMethod("RegisterHeader").Invoke(null, new object[] { header, 901 });
            connectionType.GetMethod("Connect", Type.EmptyTypes).Invoke(connection, null);
            Assert.That(connectionType.GetField("m_ConnectionThreadCancellation", BindingFlags.NonPublic | BindingFlags.Instance)
                .GetValue(connection), Is.SameAs(cancellation), "Duplicate Connect replaced the worker");
            // EditMode does not dispatch destruction callbacks for an unplayed behaviour.
            connectionType.GetMethod("OnDestroy", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(connection, null);
            UnityEngine.Object.DestroyImmediate(gameObject);
            Assert.That(cancellation.IsCancellationRequested, Is.True);
            Assert.That(headers.Contains(901), Is.False);
            Assert.That(headers.Contains(900), Is.True);
        }
        finally
        {
            if (gameObject != null) { UnityEngine.Object.DestroyImmediate(gameObject); }
            headers.Remove(900);
            headers.Remove(901);
            cancellation.Dispose();
        }
    }

    [Test]
    public void StopClearsVelocityTargetsAndBodyImmediately()
    {
        Type controllerType = FindType("SEAN.Control.VelocityController");
        var gameObject = new GameObject("reset command");
        try
        {
            Component controller = gameObject.AddComponent(controllerType);
            Rigidbody body = gameObject.AddComponent<Rigidbody>();
            var flags = BindingFlags.NonPublic | BindingFlags.Instance;
            controllerType.GetField("rb", flags).SetValue(controller, body);
            controllerType.GetField("targetLinVelocity", flags).SetValue(controller, 0.3f);
            controllerType.GetField("targetAngVelocity", flags).SetValue(controller, 0.5f);
            body.velocity = Vector3.forward;
            body.angularVelocity = Vector3.up;
            controllerType.GetMethod("Stop", flags).Invoke(controller, null);
            Assert.That(controllerType.GetField("targetLinVelocity", flags).GetValue(controller), Is.EqualTo(0f));
            Assert.That(controllerType.GetField("targetAngVelocity", flags).GetValue(controller), Is.EqualTo(0f));
            Assert.That(body.velocity, Is.EqualTo(Vector3.zero));
            Assert.That(body.angularVelocity, Is.EqualTo(Vector3.zero));
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(gameObject);
        }
    }

    private static Type FindType(string name)
    {
        return AppDomain.CurrentDomain.GetAssemblies()
            .Select(assembly => assembly.GetType(name)).First(type => type != null);
    }

    private static Vector3[] BodyVelocity(Vector3 previousPosition, Quaternion previousRotation,
        Vector3 position, Quaternion rotation, float dt)
    {
        object[] args = { previousPosition, previousRotation, position, rotation, dt, null, null };
        FindType("SEAN.TF.OdometryPublisher")
            .GetMethod("CalculateBodyVelocity", BindingFlags.NonPublic | BindingFlags.Static)
            .Invoke(null, args);
        return new[] { (Vector3)args[5], (Vector3)args[6] };
    }

    [Test]
    public void LinearVelocityUsesBodyFrame()
    {
        Quaternion heading = Quaternion.Euler(0, 90, 0);
        Vector3[] velocity = BodyVelocity(Vector3.zero, heading, Vector3.right, heading, 0.5f);
        Assert.That(Vector3.Distance(velocity[0], Vector3.forward * 2), Is.LessThan(0.0001f));
        Assert.That(velocity[1].magnitude, Is.LessThan(0.0001f));
    }

    [Test]
    public void AngularVelocityWrapsAndUsesRadians()
    {
        Vector3[] velocity = BodyVelocity(Vector3.zero, Quaternion.Euler(0, 359, 0),
            Vector3.zero, Quaternion.Euler(0, 1, 0), 0.5f);
        Assert.That(velocity[1].y, Is.EqualTo(4 * Mathf.Deg2Rad).Within(0.0001f));
    }

    [Test]
    public void IdenticalRotationsProduceFiniteZeroVelocity()
    {
        Vector3[] velocity = BodyVelocity(Vector3.one, Quaternion.identity,
            Vector3.one, Quaternion.identity, 0.02f);
        Assert.That(velocity[0], Is.EqualTo(Vector3.zero));
        Assert.That(velocity[1], Is.EqualTo(Vector3.zero));
    }

    [TestCase(0f)]
    [TestCase(90f)]
    [TestCase(180f)]
    public void SensorMountIsIndependentOfRobotHeading(float heading)
    {
        var robot = new GameObject("robot");
        var sensor = new GameObject("sensor");
        try
        {
            robot.transform.SetPositionAndRotation(new Vector3(3, 1, 5), Quaternion.Euler(20, heading, 10));
            sensor.transform.SetParent(robot.transform, false);
            sensor.transform.localPosition = new Vector3(0.3f, 0.5f, 0.2f);
            sensor.transform.localRotation = Quaternion.Euler(10, 25, 0);
            Pose pose = (Pose)FindType("SEAN.TF.RelativeTransformPublisher")
                .GetMethod("RelativePose", BindingFlags.NonPublic | BindingFlags.Static)
                .Invoke(null, new object[] { robot.transform, sensor.transform });
            Assert.That(Vector3.Distance(pose.position, sensor.transform.localPosition), Is.LessThan(0.0001f));
            Assert.That(Quaternion.Angle(pose.rotation, sensor.transform.localRotation), Is.LessThan(0.01f));
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(sensor);
            UnityEngine.Object.DestroyImmediate(robot);
        }
    }

    [Test]
    public void ClockEpochSurvivesPublisherReplacement()
    {
        Type clockType = FindType("SEAN.ROSClock.ROSClockPublisher");
        MethodInfo reset = clockType.GetMethod("ResetEpoch", BindingFlags.NonPublic | BindingFlags.Static);
        MethodInfo awake = clockType.GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance);
        reset.Invoke(null, null);
        double? epoch = null;
        try
        {
            for (int i = 0; i < 2; i++)
            {
                var gameObject = new GameObject("clock");
                try
                {
                    Component clock = gameObject.AddComponent(clockType);
                    awake.Invoke(clock, null);
                    double value = (double)clockType.GetProperty("StartMillis").GetValue(clock);
                    if (epoch.HasValue) { Assert.That(value, Is.EqualTo(epoch.Value)); }
                    epoch = value;
                }
                finally
                {
                    UnityEngine.Object.DestroyImmediate(gameObject);
                }
                System.Threading.Thread.Sleep(20);
            }
        }
        finally
        {
            reset.Invoke(null, null);
        }
    }

    [Test]
    public void LaserPrefabPublishesItsMessageFrame()
    {
        GameObject prefab = UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>(
            "Assets/Resources/SEAN/Sensors/RaycastLaserScanner.prefab");
        Component publisher = prefab.GetComponent(FindType("SEAN.Sensors.LaserScanPublisher"));
        Component transform = prefab.GetComponent(FindType("SEAN.TF.RelativeTransformPublisher"));
        Assert.That(transform, Is.Not.Null, "Scan frame must be present without Nav2 or a robot-description publisher");
        Assert.That(transform.GetType().GetField("FrameID").GetValue(transform),
            Is.EqualTo(publisher.GetType().GetField("FrameId").GetValue(publisher)));
    }

    [TestCase(640, 480, 60f, false)]
    [TestCase(800, 400, 45f, false)]
    [TestCase(640, 480, 60f, true)]
    public void CameraCalibrationUsesRenderedProjection(int width, int height, float fov, bool orthographic)
    {
        var go = new GameObject("calibration");
        var output = new RenderTexture(width, height, 24);
        try
        {
            Camera cam = go.AddComponent<Camera>();
            cam.targetTexture = output;
            cam.aspect = (float)width / height;
            cam.fieldOfView = fov;
            cam.orthographic = orthographic;
            Type rgbType = FindType("RGBCameraPublisher");
            Component publisher = go.AddComponent(rgbType);
            rgbType.GetField("offscreenCamera", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(publisher, cam);
            object info = FindType("BaseCameraPublisher").GetMethod("CameraInfo", BindingFlags.NonPublic | BindingFlags.Instance)
                .Invoke(publisher, null);
            Type infoType = info.GetType();
            Assert.That(infoType.GetField("width").GetValue(info), Is.EqualTo((uint)width));
            Assert.That(infoType.GetField("height").GetValue(info), Is.EqualTo((uint)height));
            double[] k = (double[])infoType.GetProperty("K").GetValue(info);
            if (orthographic)
            {
                Assert.That(k.All(value => value == 0), Is.True);
                return;
            }
            double focal = height / (2.0 * Math.Tan(fov * Math.PI / 360.0));
            Assert.That(k[0], Is.EqualTo(focal).Within(0.001));
            Assert.That(k[4], Is.EqualTo(focal).Within(0.001));
            Assert.That(k[2], Is.EqualTo(width / 2.0));
            Assert.That(k[5], Is.EqualTo(height / 2.0));
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(go);
            UnityEngine.Object.DestroyImmediate(output);
        }
    }

    [Test]
    public void CameraCopySkipsReadOnlyFields()
    {
        var source = new GameObject("source");
        var destination = new GameObject("destination");
        try
        {
            Camera camera = source.AddComponent<Camera>();
            camera.fieldOfView = 47f;
            MethodInfo copy = FindType("SEAN.Util.Unity").GetMethod("CopyComponent")
                .MakeGenericMethod(typeof(Camera));
            Camera result = (Camera)copy.Invoke(null, new object[] { camera, destination });
            Assert.That(result.fieldOfView, Is.EqualTo(camera.fieldOfView));
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(source);
            UnityEngine.Object.DestroyImmediate(destination);
        }
    }

    [Test]
    public void PublisherRegistersAgainAfterConnectionReplacement()
    {
        Type connectionType = FindType("Unity.Robotics.ROSTCPConnector.ROSConnection");
        Type messageType = FindType("RosMessageTypes.Std.StringMsg");
        MethodInfo publish = FindType("RosPublishExtensions").GetMethod("PublishAuto")
            .MakeGenericMethod(messageType);
        for (int i = 0; i < 2; i++)
        {
            var gameObject = new GameObject("connection");
            try
            {
                Component connection = gameObject.AddComponent(connectionType);
                object message = Activator.CreateInstance(messageType);
                Assert.DoesNotThrow(() => publish.Invoke(null, new[] { connection, "/reload_test", message }));
                object topic = connectionType.GetMethod("GetTopic").Invoke(connection, new object[] { "/reload_test" });
                Assert.That(topic.GetType().GetProperty("IsPublisher").GetValue(topic), Is.True);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(gameObject);
            }
        }
    }
}
