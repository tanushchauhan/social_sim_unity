// Copyright (c) 2026, SEAN contributors. Licensed under BSD-3-Clause.
using System;
using UnityEngine;

namespace SEAN.Metrics
{
    // Counts contact beginnings, not contact points or personal-space entries.
    public class ObjectContactSensor : MonoBehaviour
    {
        public event Action Contact;
        public uint Count { get; private set; }

        private void OnCollisionEnter(Collision collision)
        {
            Collider other = collision.collider;
            if (other.CompareTag(SEAN.AgentTag) || other.CompareTag(SEAN.GroupTag)) return;
            for (int i = 0; i < collision.contactCount; i++)
            {
                // Ignore upward support contacts (floor / ramps up to 60 degrees).
                // Side and overhead contacts still count, including during a fall.
                if (collision.GetContact(i).normal.y >= 0.5f) continue;
                Count++;
                Contact?.Invoke();
                break;
            }
        }
    }
}
