using System;
using System.Linq;
using UnityEngine;

namespace SEAN.Scenario.Agents
{
    public enum RobotResponse { Legacy, ColliderV1 }

    // ColliderV1 changes only robot response; person/person and wall forces retain their limits.
    public static class RobotAvoidance
    {
        public const float RepulsionOffset = 0f;
        public const float AnticipationGap = 3f;
        public const float SidestepForce = Parameters.A/(10*Parameters.LATERAL_DAMPENING);
        public static Collider[] BodyColliders(Rigidbody body)
        {
            if (body == null) throw new ArgumentException("Robot avoidance requires its active rigidbody");
            var colliders = body.GetComponentsInChildren<Collider>()
                .Where(c => c.enabled && !c.isTrigger && c.attachedRigidbody == body).ToArray();
            if (colliders.Length == 0) throw new ArgumentException("Robot avoidance requires physical body colliders");
            if (colliders.Any(c => !(c is BoxCollider) && !(c is SphereCollider) && !(c is CapsuleCollider)
                && !(c is WheelCollider) && !(c is MeshCollider mesh && mesh.convex)))
                throw new ArgumentException("ColliderV1 requires primitive, wheel or convex robot body shapes");
            return colliders;
        }

        public static Vector3 Force(Vector3 position, Vector3 velocity, Vector3 goalDirection,
            Rigidbody body, Collider[] colliders, out bool inFront, out bool approaching)
        {
            position.y = 0;
            goalDirection.y = 0; goalDirection.Normalize();
            float gap = float.PositiveInfinity;
            Vector3 surface = Vector3.zero;
            foreach (var collider in colliders)
            {
                if (collider == null || !collider.enabled || !collider.gameObject.activeInHierarchy) continue;
                Vector3 q;
                if (collider is WheelCollider wheel)
                {
                    Vector3 center = wheel.transform.TransformPoint(wheel.center); center.y = 0;
                    Vector3 scale = wheel.transform.lossyScale;
                    float radius = wheel.radius*Mathf.Max(Mathf.Abs(scale.x),Mathf.Abs(scale.y),Mathf.Abs(scale.z));
                    Vector3 delta = position-center;
                    q = delta.magnitude <= radius ? position : center+delta.normalized*radius;
                }
                else
                {
                    // Query at body-shape height, not the pedestrian's feet below a low robot.
                    Vector3 query = new Vector3(position.x,collider.bounds.center.y,position.z);
                    q = collider.ClosestPoint(query); q.y = 0;
                }
                float distance = Vector3.Distance(position,q);
                if (distance < gap) { gap = distance; surface = q; }
            }
            if (float.IsPositiveInfinity(gap)) throw new InvalidOperationException("Robot has no active avoidance shapes");
            Vector3 direction = position-surface;
            if (direction.sqrMagnitude < 1e-8f)
            {
                direction = position-new Vector3(body.position.x,0,body.position.z);
                if (direction.sqrMagnitude < 1e-8f) direction = -goalDirection;
            }
            direction.Normalize();
            Vector3 relativeVelocity = body.velocity-velocity; relativeVelocity.y = 0;
            inFront = Vector3.Dot(-direction,goalDirection) >= .5f;
            approaching = Vector3.Dot(goalDirection,relativeVelocity) <= 0;
            float overlap = Base.RADIUS+RepulsionOffset-gap;
            float repulsion = Parameters.A*Mathf.Exp(overlap/Parameters.B);
            Vector3 force = repulsion*direction;
            // Keep the legacy slow-walking sidestep strength, but begin before braking.
            if (gap <= AnticipationGap && inFront && approaching)
            {
                Vector3 tangent = Util.Geometry.Tangent(goalDirection);
                if (Vector3.Dot(tangent,-direction) > 0) tangent = -tangent;
                float urgency = Mathf.Clamp01((AnticipationGap-gap)/Parameters.B);
                force += urgency*Vector3.Dot(-direction,goalDirection)*SidestepForce*tangent;
            }
            return force;
        }

        public static Vector3 PreventRobotInducedReverse(Vector3 nextVelocity, Vector3 goalDirection)
        {
            goalDirection.y = 0; goalDirection.Normalize();
            float along = Vector3.Dot(nextVelocity,goalDirection);
            return along < 0 ? nextVelocity-goalDirection*along : nextVelocity;
        }
    }
}
