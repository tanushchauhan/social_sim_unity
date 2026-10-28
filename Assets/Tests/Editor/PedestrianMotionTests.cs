using System;
using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.TestTools;

public class PedestrianMotionTests
{
    private static Type FindType(string name) => AppDomain.CurrentDomain.GetAssemblies()
        .Select(a => a.GetType(name)).First(t => t != null);

    [UnityTest]
    public IEnumerator AnimatedVelocityBodyContactsStillBlockARealWall()
    {
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
        yield return new EnterPlayMode();
        float oldCapture = Time.captureDeltaTime;
        int oldRate = Application.targetFrameRate;
        GameObject avatar = null;
        var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        NavMeshDataInstance mesh = default;
        try
        {
            Time.captureDeltaTime = .02f;
            Application.targetFrameRate = -1;
            floor.transform.position = Vector3.down*.1f;
            floor.transform.localScale = new Vector3(20,.2f,20);
            wall.transform.position = Vector3.up*1.5f;
            wall.transform.localScale = new Vector3(4,3,.2f);
            var source = new NavMeshBuildSource { shape=NavMeshBuildSourceShape.Box,
                transform=Matrix4x4.TRS(floor.transform.position,Quaternion.identity,Vector3.one),
                size=floor.transform.localScale,area=0 };
            var data = NavMeshBuilder.BuildNavMeshData(NavMesh.GetSettingsByIndex(0),
                new System.Collections.Generic.List<NavMeshBuildSource> { source },
                new Bounds(Vector3.zero,new Vector3(22,5,22)),Vector3.zero,Quaternion.identity);
            Assert.That(data,Is.Not.Null);
            mesh = NavMesh.AddNavMeshData(data);
            var template = Resources.Load<GameObject>("Prefabs/RocketboxRandomAnimatedAgent")
                .GetComponent(FindType("SEAN.Scenario.Agents.RandomAvatar"));
            var choices = (GameObject[])template.GetType().GetField("avatars").GetValue(template);
            avatar = UnityEngine.Object.Instantiate(choices[7],Vector3.forward*3,Quaternion.Euler(0,180,0));
            var animator = avatar.GetComponent<Animator>();
            animator.runtimeAnimatorController = (RuntimeAnimatorController)template.GetType()
                .GetField("animationController").GetValue(template);
            var sf = (MonoBehaviour)avatar.AddComponent(FindType("IVI.SFAgent"));
            yield return null;
            var capsule = avatar.GetComponent<CapsuleCollider>();
            var material = Resources.Load<PhysicMaterial>("SEAN/Robots/VelocityControlledBody");
            Assert.That(capsule.sharedMaterial,Is.SameAs(material),"Production SFAgent startup did not apply its contact model");
            var sensor = avatar.AddComponent(FindType("SEAN.Metrics.ObjectContactSensor"));
            // Exercise the real root-motion actuator against the wall, without wall-force avoidance.
            sf.enabled = false;
            sf.StopAllCoroutines();
            animator.speed = .6f;
            animator.SetBool("Idling",false);
            animator.SetFloat("Forward",1);
            animator.SetFloat("Strafe",0);
            float minimumZ = avatar.transform.position.z;
            float until = Time.time+12;
            while (Time.time < until)
            {
                yield return null;
                minimumZ = Mathf.Min(minimumZ,avatar.transform.position.z);
            }
            Assert.That((uint)sensor.GetType().GetProperty("Count").GetValue(sensor),Is.GreaterThan(0),
                "Animated body did not actually contact the wall");
            float radius = capsule.radius*Mathf.Max(Mathf.Abs(capsule.transform.lossyScale.x),Mathf.Abs(capsule.transform.lossyScale.z));
            Assert.That(minimumZ,Is.GreaterThanOrEqualTo(.1f+radius-.01f),
                "Zero-friction animated body penetrated or crossed the physical wall");
        }
        finally
        {
            if (avatar != null)
            {
                ((IDictionary)FindType("IVI.SFAgent").GetField("GO2Agent").GetValue(null)).Remove(avatar);
                UnityEngine.Object.Destroy(avatar);
            }
            UnityEngine.Object.Destroy(floor);
            UnityEngine.Object.Destroy(wall);
            if (mesh.valid) mesh.Remove();
            Time.captureDeltaTime = oldCapture;
            Application.targetFrameRate = oldRate;
        }
    }
    [UnityTearDown] public IEnumerator LeavePlayMode()
    {
        if (Application.isPlaying) yield return new ExitPlayMode();
    }
}
