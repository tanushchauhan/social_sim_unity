// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

namespace SEAN.TF
{
    public class RelativeTransformPublisher : BaseTransformPublisher
    {
        SEAN sean;
        public string FrameID;

        public float AdjustXRot = 0;
        public float AdjustYRot = 0;
        public float AdjustZRot = 0;

        RosMessageTypes.Geometry.PoseStampedMsg baseToFramePoseStamped = new RosMessageTypes.Geometry.PoseStampedMsg();

        private void Start()
        {
            sean = SEAN.instance;
            base.Start();
            baseToFramePoseStamped.header.frame_id = FrameID;
        }

        private static Pose RelativePose(Transform parent, Transform child)
        {
            Quaternion inverse = Quaternion.Inverse(parent.rotation);
            return new Pose(inverse * (child.position - parent.position), inverse * child.rotation);
        }

        private void Update()
        {
            Pose pose = RelativePose(sean.robot.transform, transform);
            Vector3 position = pose.position;
            Quaternion rotation = pose.rotation;
            if (AdjustXRot != 0 || AdjustYRot != 0 || AdjustZRot != 0) {
                rotation *= Quaternion.Euler(AdjustXRot, AdjustYRot, AdjustZRot);
            }
            baseToFramePoseStamped.pose.position = Util.Geometry.GetGeometryPoint(position.To<FLU>());
            baseToFramePoseStamped.pose.orientation = Util.Geometry.GetGeometryQuaternion(rotation.To<FLU>());
            PublishIfNew(new NamedTransform("base_link_to_" + FrameID, baseToFramePoseStamped));
        }
    }
}
