using System;
using UnityEngine;
using SEAN.Scenario.Controllers;

namespace SEAN.Scenario.Agents
{
    public class JointAgent : Base
    {
        public string StableId { get; private set; }
        public bool Initialized { get; private set; }
        public Vector3 TargetVelocity { get; private set; }
        JointControllerDriver owner;
        protected override bool SceneControlled { get { return true; } }
        protected virtual bool UseOrca { get { return false; } }

        protected override void Start()
        {
            StableId = gameObject.name;
            base.Start();
            GetComponent<Animator>().updateMode = AnimatorUpdateMode.AnimatePhysics;
            global::SEAN.Control.VelocityBodyMaterial.Apply(rb);
            Initialized = true;
            StopMotion();
            owner = JointControllerDriver.ForScene(UseOrca);
            owner.Register(this);
            IVI.SFAgent.GO2Agent.Add(gameObject, this);
        }

        public ActorState CaptureState()
        {
            if (!Initialized) throw new InvalidOperationException("Avatar startup incomplete");
            float radius = collisionCapsule.radius * Mathf.Max(Mathf.Abs(transform.lossyScale.x), Mathf.Abs(transform.lossyScale.z));
            Vector3 desired = IsNavigating ? nearestGoalPoint - transform.position : Vector3.zero;
            desired.y = 0;
            Vector3 preferred = desired.normalized * Parameters.DESIRED_SPEED;
            return new ActorState(StableId, false, isActiveAndEnabled,
                Contract.FromUnity(rb.position), Contract.FromUnity(rb.velocity),
                Contract.HeadingFromUnity(transform.eulerAngles.y * Mathf.Deg2Rad),
                radius, Parameters.MAX_VEL, IsNavigating ? (Vector2?)Contract.FromUnity(destPos) : null,
                Contract.FromUnity(preferred));
        }

        public void SetTarget(Vector2 target) { TargetVelocity = Contract.ToUnity(target); }
        public void ApplyAtBoundary(float step)
        {
            if (isActiveAndEnabled) AnimateSceneVelocity(TargetVelocity, step);
        }
        public void StopMotion()
        {
            TargetVelocity = Vector3.zero;
            ClearSceneMotion();
        }
        protected override Vector3 UpdateVelocity()
        {
            throw new InvalidOperationException("Joint avatars must never invoke a per-avatar controller");
        }
        void OnDisable()
        {
            StopMotion();
            if (owner != null) owner.PopulationChanged();
        }
        void OnEnable()
        {
            if (owner != null) owner.PopulationChanged();
        }
        void OnDestroy()
        {
            StopMotion();
            IVI.SFAgent.GO2Agent.Remove(gameObject);
            if (owner != null) owner.Unregister(this);
        }
    }
}
