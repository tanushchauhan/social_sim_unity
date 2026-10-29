// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;

namespace SEAN.Tasks
{
    public class RandomABNav : Base
    {

        private float retryAfter;

        protected override bool NewTask()
        {
            if (Time.time < retryAfter) return false;
            if (!RandomTaskPlacement.TryPair(sean.robot.gameObject.transform,
                out Vector3 startPosition, out Vector3 goalPosition))
            {
                retryAfter = Time.time + 5f;
                Debug.LogError("RandomABNav could not find a clear, connected start/goal pair; task unchanged.");
                return false;
            }

            sean.pedestrianBehavior.SetScenarioName("RandomAB");
            robotGoal.SetActive(true);
            // Preserve the sampled surface elevation while allowing the body to settle.
            robotStart.transform.position = startPosition + Vector3.up * 0.75f;
            robotStart.transform.rotation = Util.Navmesh.RandomRotation();
            robotGoal.transform.position = goalPosition + Vector3.up * 0.5f;
            robotGoal.transform.rotation = Util.Navmesh.RandomRotation();
            SetTargetFlags(robotGoal);

            return true;
        }
    }
}