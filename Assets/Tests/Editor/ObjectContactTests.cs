using System;
using System.Collections;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.TestTools;

public class ObjectContactTests
{
    private static Type FindType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(a => a.GetType(name)).First(t => t != null);

    [Test]
    public void ObjectProximityDoesNotIncrementPhysicalContactMetric()
    {
        var context = new GameObject("inactive metric context");
        var robot = new GameObject("proximity robot");
        var obstacle = new GameObject("nearby object");
        context.SetActive(false);
        try
        {
            Type seanType = FindType("SEAN.SEAN");
            Component sean = context.AddComponent(seanType);
            // Legacy Reset is also an editor callback; this isolated context has no clock.
            LogAssert.Expect(LogType.Exception, new Regex("^NullReferenceException:"));
            Component metrics = context.AddComponent(FindType("SEAN.Metrics.Metrics"));
            seanType.GetField("_metrics", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(sean, metrics);
            robot.AddComponent<Rigidbody>();
            Type counterType = FindType("SEAN.Metrics.CountCollisions");
            Component counter = robot.AddComponent(counterType);
            counterType.GetField("sean", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(counter, sean);
            var collider = obstacle.AddComponent<BoxCollider>();
            counterType.GetMethod("OnTriggerEnter", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(counter, new object[] { collider });
            Assert.That((uint)metrics.GetType().GetProperty("ObjectCollisions").GetValue(metrics), Is.Zero);
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(robot);
            UnityEngine.Object.DestroyImmediate(obstacle);
            UnityEngine.Object.DestroyImmediate(context);
        }
    }

    [UnityTest]
    public IEnumerator VelocityMaterialPreservesSmallCommandsOnCompoundFloorBody()
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        yield return new EnterPlayMode();
        var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        var robot = new GameObject("low speed velocity body");
        try
        {
            floor.transform.position = new Vector3(0, -0.1f, 0);
            floor.transform.localScale = new Vector3(10, 0.2f, 10);
            robot.transform.position = new Vector3(0, 0.5f, 0);
            var capsule = robot.AddComponent<CapsuleCollider>();
            capsule.radius = 0.2f;
            capsule.height = 1;
            capsule.center = Vector3.up * 0.5f;
            var wheelObject = new GameObject("compound floor-contact shape");
            wheelObject.transform.SetParent(robot.transform, false);
            wheelObject.transform.localPosition = new Vector3(0, -0.4f, 0);
            var wheel = wheelObject.AddComponent<BoxCollider>();
            wheel.size = new Vector3(0.4f, 0.2f, 0.4f);
            var body = robot.AddComponent<Rigidbody>();
            body.constraints = RigidbodyConstraints.FreezeRotation;
            float until = Time.fixedTime + 0.5f;
            while (Time.fixedTime < until) yield return null;
            float start = body.position.x;
            until = Time.fixedTime + 1;
            while (Time.fixedTime < until)
            {
                body.velocity = new Vector3(1f/30f, body.velocity.y, 0);
                yield return null;
            }
            Assert.That(body.position.x-start, Is.LessThan(0.01f), "Default floor friction did not reproduce the low-speed deadzone");
            var material = Resources.Load<PhysicMaterial>("SEAN/Robots/VelocityControlledBody");
            Assert.That(material, Is.Not.Null, "Velocity body contact material is missing");
            Assert.That(material.frictionCombine, Is.EqualTo(PhysicMaterialCombine.Minimum));
            FindType("SEAN.Control.VelocityBodyMaterial").GetMethod("Apply")
                .Invoke(null, new object[] { body });
            Assert.That(wheel.sharedMaterial, Is.SameAs(material), "Compound floor-contact shape retained default friction");
            start = body.position.x;
            float begin = Time.fixedTime;
            until = begin + 1;
            while (Time.fixedTime < until)
            {
                body.velocity = new Vector3(1f/30f, body.velocity.y, 0);
                yield return null;
            }
            Assert.That(body.position.x-start, Is.EqualTo((Time.fixedTime-begin)/30f).Within(0.005f),
                        "Production material does not preserve a small commanded speed");
        }
        finally
        {
            UnityEngine.Object.Destroy(robot);
            UnityEngine.Object.Destroy(floor);
        }
    }

    [Test]
    public void VelocityMaterialRespectsCompoundBodyOwnership()
    {
        var robot = new GameObject("velocity material ownership");
        try
        {
            var body = robot.AddComponent<Rigidbody>();
            var main = robot.AddComponent<BoxCollider>();
            var child = new GameObject("compound shape");
            child.transform.SetParent(robot.transform, false);
            var shape = child.AddComponent<BoxCollider>();
            var trigger = child.AddComponent<SphereCollider>();
            trigger.isTrigger = true;
            var separate = new GameObject("independent body");
            separate.transform.SetParent(robot.transform, false);
            separate.AddComponent<Rigidbody>().isKinematic = true;
            var independent = separate.AddComponent<BoxCollider>();
            var material = (PhysicMaterial)FindType("SEAN.Control.VelocityBodyMaterial")
                .GetMethod("Apply").Invoke(null, new object[] { body });
            Assert.That(main.sharedMaterial, Is.SameAs(material));
            Assert.That(shape.sharedMaterial, Is.SameAs(material));
            Assert.That(trigger.sharedMaterial, Is.Null);
            Assert.That(independent.sharedMaterial, Is.Null);
        }
        finally { UnityEngine.Object.DestroyImmediate(robot); }
    }

    [UnityTest]
    public IEnumerator PhysicalWallContactsExcludeFloorAndPersonalSpace()
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        yield return new EnterPlayMode();
        var robot = new GameObject("contact test robot");
        var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        try
        {
            floor.transform.position = new Vector3(0, -0.1f, 0);
            floor.transform.localScale = new Vector3(10, 0.2f, 10);
            wall.transform.position = new Vector3(0.9f, 0.5f, 0);
            wall.transform.localScale = new Vector3(0.1f, 1, 4);
            robot.transform.position = new Vector3(0, 0.5f, 0);
            var capsule = robot.AddComponent<CapsuleCollider>();
            capsule.radius = 0.2f;
            capsule.height = 1;
            var personal = robot.AddComponent<SphereCollider>();
            capsule.sharedMaterial = Resources.Load<PhysicMaterial>("SEAN/Robots/VelocityControlledBody");
            personal.isTrigger = true;
            personal.radius = 1.2f;
            var body = robot.AddComponent<Rigidbody>();
            body.constraints = RigidbodyConstraints.FreezeRotation;
            Type sensorType = FindType("SEAN.Metrics.ObjectContactSensor");
            Component sensor = robot.AddComponent(sensorType);
            int notifications = 0;
            Action contact = () => notifications++;
            sensorType.GetEvent("Contact").AddEventHandler(sensor, contact);
            float until = Time.fixedTime + 0.4f;
            while (Time.fixedTime < until) yield return null;
            Assert.That(notifications, Is.Zero, "Floor support / personal-space overlap counted as object contact");
            until = Time.fixedTime + 1.4f;
            while (Time.fixedTime < until)
            {
                body.velocity = new Vector3(1, body.velocity.y, 0);
                yield return null;
            }
            Assert.That(notifications, Is.EqualTo(1), "Physical wall impact was not counted once");
            Assert.That(body.position.x, Is.LessThan(0.72f), "Velocity material allowed wall penetration");
            Assert.That((uint)sensorType.GetProperty("Count").GetValue(sensor), Is.EqualTo(1));
            until = Time.fixedTime + 0.4f;
            while (Time.fixedTime < until) yield return null;
            Assert.That(notifications, Is.EqualTo(1), "Continuing contact was counted repeatedly");
            until = Time.fixedTime + 0.7f;
            while (Time.fixedTime < until)
            {
                body.velocity = new Vector3(-1, body.velocity.y, 0);
                yield return null;
            }
            until = Time.fixedTime + 1.4f;
            while (Time.fixedTime < until)
            {
                body.velocity = new Vector3(1, body.velocity.y, 0);
                yield return null;
            }
            Assert.That(notifications, Is.EqualTo(2), "A separate second impact was lost");
            wall.tag = "NavAgent";
            until = Time.fixedTime + 0.7f;
            while (Time.fixedTime < until)
            {
                body.velocity = new Vector3(-1, body.velocity.y, 0);
                yield return null;
            }
            until = Time.fixedTime + 1.4f;
            while (Time.fixedTime < until)
            {
                body.velocity = new Vector3(1, body.velocity.y, 0);
                yield return null;
            }
            Assert.That(notifications, Is.EqualTo(2), "A pedestrian contact was counted as object contact");
        }
        finally
        {
            UnityEngine.Object.Destroy(robot);
            UnityEngine.Object.Destroy(floor);
            UnityEngine.Object.Destroy(wall);
        }
    }

    [UnityTearDown]
    public IEnumerator LeavePlayMode()
    {
        if (Application.isPlaying) yield return new ExitPlayMode();
    }
}
