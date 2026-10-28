using System;
using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.TestTools;

public class WarehouseMotionTests
{
    [UnityTest]
    public IEnumerator PlanarNav2BodyCanActuallyTurnWithoutFallingOrTilting()
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
        yield return new EnterPlayMode();
        var go = new GameObject("Nav2 fixture body");
        float capture = Time.captureDeltaTime;
        int rate = Application.targetFrameRate;
        try
        {
            Time.captureDeltaTime = .02f;
            Application.targetFrameRate = -1;
            go.transform.position = new Vector3(0,2,0);
            var body = go.AddComponent<Rigidbody>();
            Physics.SyncTransforms();
            Assert.That(body.position.y,Is.EqualTo(2).Within(.001f));
            var fixture = AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("SEAN.Scenario.Agents.WarehouseEncounterValidation")).First(t => t != null);
            fixture.GetMethod("ConfigureMotion").Invoke(null,new object[] { body,true });
            for (int i = 0; i < 30; i++)
            {
                // Same velocity/angular-velocity actuation used by VelocityController.
                body.velocity = new Vector3(.3f,1,0);
                body.angularVelocity = new Vector3(1,1,1);
                yield return null;
            }
            Assert.That(Mathf.Abs(Mathf.DeltaAngle(0,body.rotation.eulerAngles.y)),Is.GreaterThan(15),
                "Fixture suppressed the commanded Nav2 yaw");
            Assert.That(Mathf.Abs(Mathf.DeltaAngle(0,body.rotation.eulerAngles.x)),Is.LessThan(.1f));
            Assert.That(Mathf.Abs(Mathf.DeltaAngle(0,body.rotation.eulerAngles.z)),Is.LessThan(.1f));
            Assert.That(body.position.y,Is.EqualTo(2).Within(.001f));
            Assert.That(body.position.x,Is.GreaterThan(.1f));
        }
        finally
        {
            UnityEngine.Object.Destroy(go);
            Time.captureDeltaTime = capture;
            Application.targetFrameRate = rate;
        }
    }
    [UnityTearDown] public IEnumerator LeavePlayMode()
    {
        if (Application.isPlaying) yield return new ExitPlayMode();
    }
}
