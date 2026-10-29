using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace SEAN.TF
{
    public class OdometryPublisher : MonoBehaviour
    {
        private ROSConnection ros;
        [Tooltip("Publication interval in simulation seconds.")]
        public float publishMessageFrequency = 0.05f;
        public string topicName;
        public Transform PublishedTransform;
        public string FrameId = "map";
        public string ChildFrameId = "base_link";

        private RosMessageTypes.Nav.OdometryMsg message;
        private Tasks.Base task;
        private float timeElapsed;
        private Vector3 previousPosition;
        private Quaternion previousRotation;
        private bool hasPreviousSample;

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            message = new RosMessageTypes.Nav.OdometryMsg();
            task = SEAN.instance.robotTask;
            if (task != null)
            {
                task.onNewTask += ResetOdometry;
            }
            ResetOdometry();
        }

        private void OnEnable()
        {
            ResetOdometry();
        }

        private void OnDestroy()
        {
            if (task != null)
            {
                task.onNewTask -= ResetOdometry;
            }
        }

        public void ResetOdometry()
        {
            hasPreviousSample = false;
            timeElapsed = 0;
        }

        private static void CalculateBodyVelocity(
            Vector3 previousPosition, Quaternion previousRotation,
            Vector3 position, Quaternion rotation, float deltaTime,
            out Vector3 linear, out Vector3 angular)
        {
            linear = Quaternion.Inverse(rotation) * ((position - previousPosition) / deltaTime);
            Quaternion delta = rotation * Quaternion.Inverse(previousRotation);
            delta.ToAngleAxis(out float angle, out Vector3 axis);
            if (angle > 180f) { angle -= 360f; }
            angular = Mathf.Abs(angle) < 0.0001f
                ? Vector3.zero
                : Quaternion.Inverse(rotation) * (axis * (angle * Mathf.Deg2Rad / deltaTime));
        }

        private void FixedUpdate()
        {
            if (PublishedTransform == null || message == null) { return; }
            Vector3 position = PublishedTransform.position;
            Quaternion rotation = PublishedTransform.rotation;
            float deltaTime = Time.fixedDeltaTime;
            if (!hasPreviousSample || deltaTime <= 0)
            {
                previousPosition = position;
                previousRotation = rotation;
                hasPreviousSample = true;
                return;
            }

            CalculateBodyVelocity(previousPosition, previousRotation, position, rotation,
                deltaTime, out Vector3 linear, out Vector3 angular);
            previousPosition = position;
            previousRotation = rotation;
            timeElapsed += deltaTime;
            if (timeElapsed < publishMessageFrequency) { return; }

            SEAN.instance.clock.UpdateMHeader(message.header);
            message.header.frame_id = FrameId;
            message.child_frame_id = ChildFrameId;
            message.pose.pose.position = Util.Geometry.GetGeometryPoint(position.To<FLU>());
            message.pose.pose.orientation = Util.Geometry.GetGeometryQuaternion(rotation.To<FLU>());
            message.twist.twist.linear = Util.Geometry.GetGeometryVector3(linear.To<FLU>());
            // Angular velocity changes sign when converting Unity's handedness to ROS.
            message.twist.twist.angular = Util.Geometry.GetGeometryVector3((-angular).To<FLU>());
            ros.PublishAuto(topicName, message);
            timeElapsed = publishMessageFrequency > 0 ? timeElapsed % publishMessageFrequency : 0;
        }
    }
}
