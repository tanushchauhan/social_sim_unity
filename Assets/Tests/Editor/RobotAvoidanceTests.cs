using System;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.AI;

public class RobotAvoidanceTests
{
    private static Type Helper => AppDomain.CurrentDomain.GetAssemblies()
        .Select(a => a.GetType("SEAN.Scenario.Agents.RobotAvoidance")).First(t => t != null);
    private GameObject robot;
    private Rigidbody body;
    private CapsuleCollider shape;

    [SetUp] public void SetUp()
    {
        robot = new GameObject("robot force fixture");
        body = robot.AddComponent<Rigidbody>(); body.isKinematic = true;
        shape = robot.AddComponent<CapsuleCollider>(); shape.radius = .16f; shape.height = .46f;
        shape.center = Vector3.up*.19f;
        Physics.SyncTransforms();
    }
    [TearDown] public void TearDown() { UnityEngine.Object.DestroyImmediate(robot); }
    private Vector3 Force(Vector3 p, Vector3 v, Vector3 goal)
    {
        Physics.SyncTransforms();
        object[] args = { p, v, goal, body, new Collider[] { shape }, 80f, false, false };
        return (Vector3)Helper.GetMethod("Force").Invoke(null,args);
    }

    [Test] public void ParkedRobotFacingDoesNotChangeCircularBodyForce()
    {
        Vector3 p = new Vector3(.3f,0,1.2f);
        var before = Force(p,Vector3.back*.6f,Vector3.back);
        robot.transform.rotation = Quaternion.Euler(0,180,0);
        var after = Force(p,Vector3.back*.6f,Vector3.back);
        Assert.That(Vector3.Distance(before,after),Is.LessThan(.001f));
    }
    [Test] public void OffAxisRobotSidestepMirrorsAwayFromItsBody()
    {
        var left = Force(new Vector3(-.3f,0,1.2f),Vector3.back*.6f,Vector3.back);
        var right = Force(new Vector3(.3f,0,1.2f),Vector3.back*.6f,Vector3.back);
        Assert.That(left.x,Is.LessThan(0)); Assert.That(right.x,Is.GreaterThan(0));
        Assert.That(left.x,Is.EqualTo(-right.x).Within(.001f));
        Assert.That(left.z,Is.EqualTo(right.z).Within(.001f));
        Assert.That(left.z,Is.GreaterThan(0),"Repulsion must brake a southbound pedestrian");
    }
    [Test] public void ApproachUsesActualRobotVelocity()
    {
        body.isKinematic = false; body.useGravity = false;
        body.velocity = Vector3.forward*.3f;
        Vector3 closing = Force(Vector3.forward,Vector3.zero,Vector3.back);
        body.velocity = Vector3.back*.3f;
        Vector3 leaving = Force(Vector3.forward,Vector3.zero,Vector3.back);
        Assert.That(Mathf.Abs(closing.x),Is.GreaterThan(1));
        Assert.That(leaving.x,Is.EqualTo(0).Within(.001f));
    }
    [Test] public void SidestepBuildsBeforeFullBrakingWithoutExceedingItsLimit()
    {
        float far = Mathf.Abs(Force(Vector3.forward*3.5f,Vector3.back*.6f,Vector3.back).x);
        float early = Mathf.Abs(Force(Vector3.forward*2.5f,Vector3.back*.6f,Vector3.back).x);
        float near = Mathf.Abs(Force(Vector3.forward*.8f,Vector3.back*.6f,Vector3.back).x);
        Assert.That(far,Is.EqualTo(0).Within(.001f));
        Assert.That(early,Is.InRange(5f,10f),"Pedestrian must start passing before it brakes to a stop");
        Assert.That(near,Is.EqualTo(10f).Within(.001f));
    }
    [Test] public void LargerColliderRepelsEarlierAndFiniteAtCoincidence()
    {
        float small = Force(Vector3.forward,Vector3.back*.6f,Vector3.back).z;
        shape.radius = .3f; shape.height = .7f;
        float large = Force(Vector3.forward,Vector3.back*.6f,Vector3.back).z;
        Assert.That(large,Is.GreaterThan(small));
        Vector3 atCenter = Force(Vector3.zero,Vector3.zero,Vector3.back);
        Assert.That(float.IsNaN(atCenter.sqrMagnitude) || float.IsInfinity(atCenter.sqrMagnitude),Is.False);
        Assert.That(atCenter.z,Is.GreaterThan(0));
    }
    [Test] public void PhysicalShapeSelectionExcludesTriggersAndIndependentBodies()
    {
        var trigger = robot.AddComponent<SphereCollider>(); trigger.isTrigger = true; trigger.radius=10;
        var child = new GameObject("independent"); child.transform.SetParent(robot.transform,false);
        child.AddComponent<Rigidbody>(); child.AddComponent<BoxCollider>();
        var selected = (Collider[])Helper.GetMethod("BodyColliders").Invoke(null,new object[] { body });
        Assert.That(selected,Is.EqualTo(new Collider[] { shape }));
    }
    [Test] public void ReverseClampPreservesStoppingAndLateralMotion()
    {
        var result = (Vector3)Helper.GetMethod("PreventRobotInducedReverse")
            .Invoke(null,new object[] { new Vector3(.2f,0,.3f), Vector3.back });
        Assert.That(result,Is.EqualTo(new Vector3(.2f,0,0)));
    }
    [Test] public void ProductionForceRetainsRobotBrakingAfterLegacyLimits()
    {
        var go = new GameObject("inactive pedestrian force fixture"); go.SetActive(false);
        try
        {
            Type type = AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("IVI.SFAgent")).First(t => t != null);
            Component agent = go.AddComponent(type);
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
            type.GetField("robotBody",flags).SetValue(agent,body);
            type.GetField("robotColliders",flags).SetValue(agent,new Collider[] { shape });
            Type baseType = type.BaseType;
            baseType.GetField("nmPath",flags).SetValue(agent,new NavMeshPath());
            baseType.GetField("destPos",flags).SetValue(agent,Vector3.back*3);
            go.transform.position = Vector3.forward*.5f;
            go.transform.rotation = Quaternion.Euler(0,180,0);
            object force = type.GetMethod("ComputeForce",flags).Invoke(agent,null);
            var actual = (Vector3)force.GetType().GetField("force").GetValue(force);
            Assert.That(actual.z,Is.GreaterThan(0),"Full production force stripped the robot's braking component");
        }
        finally { UnityEngine.Object.DestroyImmediate(go); }
    }
}
