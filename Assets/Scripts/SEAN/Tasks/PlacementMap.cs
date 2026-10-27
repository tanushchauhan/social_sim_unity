using System;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace SEAN.Tasks
{
    // A versioned snapshot of the supported ROS map, in ROS FLU coordinates.
    // It constrains placement; it does not claim that the map matches all colliders.
    [Serializable]
    public class PlacementMap
    {
        public int width, height;
        public float resolution, originX, originY;
        public string freeCells, source, sourceCommit, yamlSha256, imageSha256;
        [NonSerialized] private byte[] cells;
        private static PlacementMap warehouse;

        public static bool Allows(Vector3 point)
        {
            if (SceneManager.GetActiveScene().name != "Warehouse") return true;
            if (warehouse == null)
            {
                TextAsset asset = Resources.Load<TextAsset>("SEAN/Placement/Warehouse");
                if (asset == null) return false;
                warehouse = JsonUtility.FromJson<PlacementMap>(asset.text);
            }
            return warehouse.IsFree(point.z, -point.x);
        }

        public bool IsFree(float x, float y)
        {
            if (width <= 0 || height <= 0 || resolution <= 0 || string.IsNullOrEmpty(freeCells)) return false;
            if (cells == null) cells = Convert.FromBase64String(freeCells);
            if (cells.Length != (width*height+7)/8) return false;
            int cx = Mathf.FloorToInt((x-originX)/resolution);
            int cy = Mathf.FloorToInt((y-originY)/resolution);
            // Include one cell beyond the independent 0.6 m ROS test footprint.
            int radius = Mathf.CeilToInt(0.7f/resolution);
            for (int dy = -radius; dy <= radius; dy++)
            for (int dx = -radius; dx <= radius; dx++)
            {
                if (dx*dx+dy*dy > radius*radius) continue;
                int ix = cx+dx, iy = cy+dy;
                if (ix < 0 || ix >= width || iy < 0 || iy >= height) return false;
                int index = iy*width+ix;
                if ((cells[index/8] & (1 << (index%8))) == 0) return false;
            }
            return true;
        }
    }
}
