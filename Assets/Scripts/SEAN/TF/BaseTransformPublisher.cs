// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;

namespace SEAN.TF
{
    // Collects one frame's transforms so /tf crosses the TCP bridge as one message
    // instead of one message per transform; per-message endpoint cost caused a growing backlog.
    public static class TransformBatch
    {
        private static readonly List<RosMessageTypes.Geometry.TransformStampedMsg> pending =
            new List<RosMessageTypes.Geometry.TransformStampedMsg>();

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        public static void Clear() { pending.Clear(); }

        public static void Add(RosMessageTypes.Geometry.TransformStampedMsg transform) { pending.Add(transform); }

        public static RosMessageTypes.Tf2.TFMessageMsg Take()
        {
            if (pending.Count == 0) { return null; }
            var message = new RosMessageTypes.Tf2.TFMessageMsg(pending.ToArray());
            pending.Clear();
            return message;
        }
    }

    public class BaseTransformPublisher : MonoBehaviour
    {
        private ROSConnection ros;
        private RosMessageTypes.BuiltinInterfaces.TimeMsg LastHeader = new RosMessageTypes.BuiltinInterfaces.TimeMsg();

        protected void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
        }

        public class NamedTransform
        {
            public string name;
            public RosMessageTypes.Geometry.PoseStampedMsg pose;

            public NamedTransform(string name, RosMessageTypes.Geometry.PoseStampedMsg pose)
            {
                this.name = name;
                this.pose = pose;
            }
        }

        protected void PublishIfNew(NamedTransform transform)
        {
            List<NamedTransform> transforms = new List<NamedTransform>();
            transforms.Add(transform);
            PublishIfNew(transforms);
        }

        protected void PublishIfNew(List<NamedTransform> transforms)
        {
            foreach (NamedTransform transform in transforms)
            {
                SEAN.instance.clock.UpdateMHeader(transform.pose.header);
                var stamp = transform.pose.header.stamp;
                if (LastHeader.sec > stamp.sec || (LastHeader.sec == stamp.sec && LastHeader.nanosec >= stamp.nanosec))
                {
                    return;
                }
                // Names are "<parent>_to_<child>", for example "map_to_odom" or "base_link_to_laser".
                string[] frames = transform.name.TrimStart('/').Split(new[] { "_to_" }, 2, System.StringSplitOptions.None);
                var tf = new RosMessageTypes.Geometry.TransformStampedMsg();
                tf.header.stamp = stamp;
                tf.header.frame_id = frames[0];
                tf.child_frame_id = frames[1];
                tf.transform.translation.x = transform.pose.pose.position.x;
                tf.transform.translation.y = transform.pose.pose.position.y;
                tf.transform.translation.z = transform.pose.pose.position.z;
                tf.transform.rotation = transform.pose.pose.orientation;
                TransformBatch.Add(tf);
            }
            LastHeader.sec = transforms[0].pose.header.stamp.sec;
            LastHeader.nanosec = transforms[0].pose.header.stamp.nanosec;
        }

        // Every Update has run before the first LateUpdate, so one publisher sends the whole frame.
        protected void LateUpdate()
        {
            var message = TransformBatch.Take();
            if (message != null) { ros.PublishAuto("/tf", message); }
        }
    }
}
