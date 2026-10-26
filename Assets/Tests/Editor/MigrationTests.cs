using System;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

public class MigrationTests
{
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
