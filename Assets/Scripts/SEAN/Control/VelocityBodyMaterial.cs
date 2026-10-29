// Copyright (c) 2026, SEAN contributors. Licensed under BSD-3-Clause.
using UnityEngine;

namespace SEAN.Control
{
    public static class VelocityBodyMaterial
    {
        public static PhysicMaterial Apply(Rigidbody body)
        {
            var material = Resources.Load<PhysicMaterial>("SEAN/Robots/VelocityControlledBody");
            if (body == null || material == null)
                throw new System.ArgumentException("Velocity body requires a rigidbody and contact material");
            foreach (Collider shape in body.GetComponentsInChildren<Collider>(true))
                if (!shape.isTrigger && shape.attachedRigidbody == body)
                    shape.sharedMaterial = material;
            return material;
        }
    }
}
