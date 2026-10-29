using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

namespace SEAN.Sensors
{
    // Opt-in measured target for validating the real robot publishers in a player.
    public class SensorValidationFixture : MonoBehaviour
    {
        [Serializable] public class CameraRecord
        {
            public string topic, frame;
            public Vector3 position, forward, up;
            public Quaternion rotation;
            public float far, fov;
        }
        [Serializable] public class Record
        {
            public float heading;
            public Vector3 robotPosition, wallPosition, wallNormal;
            public CameraRecord[] cameras;
            public Vector3 laserPosition, laserForward;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (!System.Environment.GetCommandLineArgs().Contains("-sensor-validation-output")) return;
            var go = new GameObject("Measured sensor validation fixture");
            DontDestroyOnLoad(go);
            go.AddComponent<SensorValidationFixture>();
        }

        private IEnumerator Start()
        {
            while (SEAN.instance == null || SEAN.instance.robot == null) yield return null;
            yield return new WaitForSeconds(8);
            string[] args = System.Environment.GetCommandLineArgs();
            string output = args[Array.IndexOf(args, "-sensor-validation-output") + 1];
            int headingIndex = Array.IndexOf(args, "-sensor-validation-heading");
            float heading = headingIndex < 0 ? 0 : float.Parse(args[headingIndex + 1], CultureInfo.InvariantCulture);
            var robot = SEAN.instance.robot;
            SEAN.instance.robotTask.enabled = false;
            foreach (var controller in FindObjectsOfType<global::SEAN.Control.VelocityController>()) controller.enabled = false;
            foreach (var body in robot.GetComponentsInChildren<Rigidbody>())
            {
                body.velocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
                body.useGravity = false;
                body.constraints = RigidbodyConstraints.FreezeAll;
            }
            robot.transform.SetPositionAndRotation(new Vector3(1000, 100, 1000), Quaternion.Euler(0, heading, 0));
            Camera rgb = robot.camera_first;
            // Near face is exactly two metres ahead of the first-person camera.
            Vector3 wallNormal = robot.transform.forward;
            Vector3 wallPosition = rgb.transform.position + wallNormal * 2;
            GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            wall.name = "Measured two metre wall";
            wall.transform.SetPositionAndRotation(wallPosition + wallNormal * 0.05f, robot.rotation);
            wall.transform.localScale = new Vector3(8, 6, 0.1f);
            Color[] colors = { Color.red, Color.green, Color.blue, Color.yellow };
            for (int i = 0; i < 4; ++i)
            {
                GameObject panel = GameObject.CreatePrimitive(PrimitiveType.Cube);
                panel.name = "Orientation panel " + i;
                panel.transform.SetPositionAndRotation(wallPosition + rgb.transform.right * (i % 2 == 0 ? -1 : 1)
                    + rgb.transform.up * (i < 2 ? 0.8f : -0.8f) - wallNormal * 0.02f, robot.rotation);
                panel.transform.localScale = new Vector3(0.5f, 0.5f, 0.02f);
                panel.GetComponent<Renderer>().material = new Material(Resources.Load<Shader>("SEAN/Sensors/SensorValidationColor")) { color = colors[i] };
                Destroy(panel.GetComponent<Collider>());
            }
            Physics.SyncTransforms();
            var laser = robot.GetComponentInChildren<RaycastLaserScanner>();
            var publishers = robot.GetComponentsInChildren<BaseCameraPublisher>();
            Record record = new Record {
                heading = heading, robotPosition = robot.position,
                wallPosition = wallPosition, wallNormal = wallNormal,
                laserPosition = laser.transform.position, laserForward = laser.transform.forward,
                cameras = publishers.Select(p => {
                    Camera c = p.GetComponent<Camera>();
                    return new CameraRecord { topic = p.Topic, frame = p.FrameId, position = c.transform.position,
                        forward = c.transform.forward, up = c.transform.up, rotation = c.transform.rotation, far = c.farClipPlane, fov = c.fieldOfView };
                }).ToArray()
            };
            yield return new WaitForSeconds(4);
            File.WriteAllText(output + ".tmp", JsonUtility.ToJson(record, true));
            File.Move(output + ".tmp", output);
            Debug.Log("Measured sensor validation fixture ready: " + output);
        }
    }
}
