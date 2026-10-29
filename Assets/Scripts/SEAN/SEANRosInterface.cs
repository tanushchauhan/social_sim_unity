
// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;

namespace SEAN
{
    public class SEANRosInterface: MonoBehaviour
    {
        protected ROSConnection ros;
        protected SEAN sean;
        public string TopicTaskNew = "/social_sim/control/task/new";
        public string TopicSceneReload = "/social_sim/control/scene/reload";

        protected void Start()
        {
            sean = SEAN.instance;
            ros = ROSConnection.GetOrCreateInstance();

            ros.Subscribe<RosMessageTypes.Std.BoolMsg>(TopicTaskNew, RecvTaskNew);
            ros.Subscribe<RosMessageTypes.Std.BoolMsg>(TopicSceneReload, RecvSceneReload);
        }

        private void OnDestroy()
        {
            if (ros == null) { return; }
            ros.Unsubscribe(TopicTaskNew);
            ros.Unsubscribe(TopicSceneReload);
        }

        private void RecvSceneReload(RosMessageTypes.Std.BoolMsg msg)
        {
            if (msg.data && !sean.EvaluationMode)
                SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
        }

        void RecvTaskNew(RosMessageTypes.Std.BoolMsg msg)
        {
            if (sean.EvaluationMode) {
                Debug.LogWarning("New Task disabled in evaluation mode");
                return;
            }
            if (msg.data) {
                sean.robotTask.StartNewTask();
            }
        }
    }
}