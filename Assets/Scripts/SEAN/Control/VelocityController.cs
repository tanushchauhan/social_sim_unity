// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;

namespace SEAN.Control
{
    public class VelocityController : ControlSubscriber
    {
        private Rigidbody rb;
        private Tasks.Base task;
        private float targetLinVelocity;
        private float targetAngVelocity;
        private float lastMessageTS;
        public float maxTimeDeltaSec = 0.25f;

        protected void Start()
        {
            base.Start();
            rb = sean.robot.base_link.GetComponent<Rigidbody>();
            task = sean.robotTask;
            task.onNewTask += Stop;
        }

        private void OnDisable()
        {
            Stop();
        }

        private void OnDestroy()
        {
            if (task != null) { task.onNewTask -= Stop; }
        }

        private void Stop()
        {
            targetLinVelocity = targetAngVelocity = 0;
            if (rb == null) { return; }
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        private void FixedUpdate()
        {
            if (Time.time - lastMessageTS > maxTimeDeltaSec)
                targetAngVelocity = targetLinVelocity = 0;

            rb.angularVelocity = Vector3.up * -targetAngVelocity;
            Vector3 velocity = rb.transform.forward * targetLinVelocity;
            velocity.y = rb.velocity.y;
            rb.velocity = velocity;
        }

        protected sealed override void CmdVelMessage(RosMessageTypes.Geometry.TwistMsg msg)
        {
            if (msg == null || rb == null) { return; }
            targetLinVelocity = (float)msg.linear.x;
            targetAngVelocity = (float)msg.angular.z;
            lastMessageTS = Time.time;
        }
    }
}
