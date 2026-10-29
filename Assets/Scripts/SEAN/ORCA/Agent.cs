using UnityEngine;

namespace SEAN.ORCA
{
    public class Agent : Scenario.Agents.JointAgent
    {
        // Local avoidance is coordinated once per scene, never once per agent.
        protected override bool UseOrca { get { return true; } }
    }
}
