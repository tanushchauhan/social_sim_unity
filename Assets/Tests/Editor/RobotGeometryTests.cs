using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

public class RobotGeometryTests
{
    [Test]
    public void KuriPhysicalBodyFitsNavigationEnvelope()
    {
        var root = PrefabUtility.LoadPrefabContents("Assets/Resources/SEAN/Robots/Kuri.prefab");
        try
        {
            Physics.SyncTransforms();
            var body = root.GetComponentsInChildren<Rigidbody>(true).Single(r => r.name == "base_link");
            var shapes = body.GetComponentsInChildren<Collider>(true)
                .Where(c => !c.isTrigger && c.enabled && c.attachedRigidbody == body).ToArray();
            Assert.That(shapes.Length, Is.GreaterThan(0));
            float maximum = 0;
            foreach (var shape in shapes)
            {
                Bounds b = shape.bounds;
                foreach (float x in new[] { b.min.x, b.max.x })
                    foreach (float z in new[] { b.min.z, b.max.z })
                        maximum = Mathf.Max(maximum, new Vector2(x-body.position.x, z-body.position.z).magnitude);
                if (shape is WheelCollider wheel)
                {
                    // WheelCollider.bounds can be empty outside PlayMode.
                    Vector3 center = wheel.transform.TransformPoint(wheel.center);
                    Vector3 scale = wheel.transform.lossyScale;
                    float radius = wheel.radius * Mathf.Max(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z));
                    foreach (var p in new[] { center, center-wheel.transform.up*wheel.suspensionDistance })
                        maximum = Mathf.Max(maximum,
                            new Vector2(p.x-body.position.x, p.z-body.position.z).magnitude+radius);
                }
                TestContext.Out.WriteLine(shape.GetType().Name + " " + shape.name + ": " + b);
            }
            TestContext.Out.WriteLine("Kuri horizontal collider AABB corner envelope (m): " + maximum);
            Assert.That(maximum, Is.GreaterThan(0.1f), "Collider bounds were not populated");
            Assert.That(maximum, Is.LessThanOrEqualTo(0.25f), "Kuri exceeds the body envelope used by the navigation profile");
        }
        finally { PrefabUtility.UnloadPrefabContents(root); }
    }
}
