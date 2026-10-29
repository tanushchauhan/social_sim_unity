// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Mohamed Hussein
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace SEAN.Scenario
{
    public class Publisher : MonoBehaviour
    {
        public string Topic = "/social_sim/scene_info";
        ROSConnection ros;
        SEAN sean;

        public float PublishInterval = 1f;
        private float lastPublishTime = float.NegativeInfinity;
        private RosMessageTypes.SocialSimRos.SceneInfoMsg message;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            sean = SEAN.instance;
            message = new RosMessageTypes.SocialSimRos.SceneInfoMsg();
        }

        private void Update()
        {
            if (!sean.robotTask.isRunning || Time.time - lastPublishTime < PublishInterval) { return; }
            lastPublishTime = Time.time;
            sean.clock.UpdateMHeader(message.header);
            message.header.frame_id = "map";
            message.scenario_name = sean.pedestrianBehavior.scenario_name;
            message.robot_start_pose = Util.Geometry.GetMPose(sean.robotTask.robotStart);
            message.robot_target_pose = Util.Geometry.GetMPose(sean.robotTask.robotGoal);
            message.num_people = (ushort)sean.pedestrianBehavior.agents.Length;
            message.num_groups = (ushort)sean.pedestrianBehavior.groups.Length;
            message.environment = sean.environment.name;
            ros.PublishAuto(Topic, message);
        }
    }
}
