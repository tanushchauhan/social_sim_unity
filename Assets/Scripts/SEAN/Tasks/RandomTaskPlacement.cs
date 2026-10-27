using UnityEngine;
using UnityEngine.AI;

namespace SEAN.Tasks
{
    // RandomABNav's robot/camera envelope, separate from pedestrian sampling.
    public static class RandomTaskPlacement
    {
        public const float Clearance = 0.65f;
        public const float MinimumSeparation = 3f;
        public const int MaximumAttempts = 256;

        public static bool IsClear(Vector3 point, Transform ignoredRobot)
        {
            if (!PlacementMap.Allows(point)) return false;
            if (!NavMesh.SamplePosition(point, out NavMeshHit surface, 0.1f, NavMesh.AllAreas)
                || Vector3.Distance(surface.position, point) > 0.1f
                || !NavMesh.FindClosestEdge(surface.position, out NavMeshHit edge, NavMesh.AllAreas)
                || edge.distance < Clearance) return false;

            // Include the space occupied by the robot and overhead-camera column (mount at 3.5 m),
            // not just the baked pedestrian mesh. Leave the supporting floor below.
            foreach (Collider collider in Physics.OverlapBox(point + Vector3.up * 1.9f,
                new Vector3(Clearance, 1.8f, Clearance), Quaternion.identity,
                ~0, QueryTriggerInteraction.Ignore))
            {
                if (ignoredRobot != null && collider.transform.IsChildOf(ignoredRobot)) continue;
                return false;
            }
            return true;
        }

        public static bool TryPair(Transform ignoredRobot, out Vector3 start, out Vector3 goal)
        {
            start = goal = Vector3.zero;
            Physics.SyncTransforms();
            NavMeshTriangulation mesh = NavMesh.CalculateTriangulation();
            if (mesh.indices.Length < 3) return false;
            float[] cumulative = new float[mesh.indices.Length / 3];
            float area = 0;
            for (int t = 0; t < cumulative.Length; t++)
            {
                Vector3 a = mesh.vertices[mesh.indices[3*t]];
                Vector3 b = mesh.vertices[mesh.indices[3*t+1]];
                Vector3 c = mesh.vertices[mesh.indices[3*t+2]];
                area += Vector3.Cross(b-a, c-a).magnitude * 0.5f;
                cumulative[t] = area;
            }
            if (area <= 0 || float.IsInfinity(area) || float.IsNaN(area)) return false;
            var path = new NavMeshPath();
            for (int attempt = 0; attempt < MaximumAttempts; attempt++)
            {
                if (!TryPoint(mesh, cumulative, ignoredRobot, out Vector3 a)
                    || !TryPoint(mesh, cumulative, ignoredRobot, out Vector3 b)
                    || Vector3.Distance(a, b) < MinimumSeparation
                    || !NavMesh.CalculatePath(a, b, NavMesh.AllAreas, path)
                    || path.status != NavMeshPathStatus.PathComplete) continue;
                start = a;
                goal = b;
                return true;
            }
            return false;
        }

        private static bool TryPoint(NavMeshTriangulation mesh, float[] cumulative,
            Transform ignoredRobot, out Vector3 point)
        {
            float selected = Random.value * cumulative[cumulative.Length - 1];
            int triangle = System.Array.BinarySearch(cumulative, selected);
            if (triangle < 0) triangle = ~triangle;
            triangle = Mathf.Min(triangle, cumulative.Length - 1);
            Vector3 a = mesh.vertices[mesh.indices[3*triangle]];
            Vector3 b = mesh.vertices[mesh.indices[3*triangle+1]];
            Vector3 c = mesh.vertices[mesh.indices[3*triangle+2]];
            // Uniform barycentric sampling within one area-weighted triangle.
            float u = Mathf.Sqrt(Random.value), v = Random.value;
            point = (1-u)*a + u*(1-v)*b + u*v*c;
            return IsClear(point, ignoredRobot);
        }
    }
}
