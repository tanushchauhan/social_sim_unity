using System;
using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

public class LaserClearingTests
{
    [Test]
    public void ProductionLaserPrefabDefaultsToFreeSpaceReturns()
    {
        var prefab=PrefabUtility.LoadPrefabContents("Assets/Resources/SEAN/Sensors/RaycastLaserScanner.prefab");
        try
        {
            var type=AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("SEAN.Sensors.RaycastLaserScanner")).First(t => t != null);
            var scanner=prefab.GetComponent(type);
            Assert.That(type.GetField("noReturn").GetValue(scanner).ToString(),Is.EqualTo("PositiveInfinity"));
        }
        finally { PrefabUtility.UnloadPrefabContents(prefab); }
    }
    [UnityTest]
    public IEnumerator DisappearingPhysicalObstacleBecomesFreeRayButTooCloseHitStaysInvalid()
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
        yield return new EnterPlayMode();
        var source = new GameObject("Laser probe");
        var obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        try
        {
            source.transform.position = Vector3.up;
            obstacle.transform.position = new Vector3(0,1,2);
            obstacle.transform.localScale = new Vector3(.5f,1,.5f);
            var type = AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetType("SEAN.Sensors.RaycastLaserScanner")).First(t => t != null);
            var scanner = source.AddComponent(type);
            type.GetField("samples").SetValue(scanner,1);
            Physics.SyncTransforms();
            yield return null;
            Func<float> scan = () => ((float[])type.GetMethod("Scan").Invoke(scanner,null))[0];
            Assert.That(scan(),Is.EqualTo(1.75f).Within(.01f));
            obstacle.SetActive(false);Physics.SyncTransforms();yield return null;
            Assert.That(float.IsPositiveInfinity(scan()),Is.True,"Empty beam cannot clear an old pedestrian mark");
            obstacle.transform.position = new Vector3(0,1,.05f);
            obstacle.transform.localScale = new Vector3(.1f,.1f,.04f);
            obstacle.SetActive(true);Physics.SyncTransforms();yield return null;
            Assert.That(scan(),Is.EqualTo(0),"Too-close hit must not claim free space through an object");
            obstacle.SetActive(false);Physics.SyncTransforms();yield return null;
            var mode = type.GetField("noReturn");
            mode.SetValue(scanner,Enum.Parse(mode.FieldType,"LegacyZero"));
            Assert.That(scan(),Is.EqualTo(0),"Legacy control must preserve old missing-return behavior");
        }
        finally { UnityEngine.Object.Destroy(source);UnityEngine.Object.Destroy(obstacle); }
    }
    [UnityTearDown] public IEnumerator LeavePlayMode()
    {
        if (Application.isPlaying) yield return new ExitPlayMode();
    }
}
