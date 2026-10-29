using System;
using System.IO;
using UnityEditor;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace SEAN.Editor
{
    /// <summary>
    /// Generates C# classes for the SEAN ROS 2 messages into Assets/RosMessages.
    /// Run from the menu, or headless with -executeMethod SEAN.Editor.GenerateRosMessages.Generate.
    /// The social_sim_ros package directory comes from SEAN_ROS_PACKAGE_DIR (default ../social_sim_ros/social_sim_ros).
    /// </summary>
    public static class GenerateRosMessages
    {
        const string OutPath = "Assets/RosMessages";

        [MenuItem("SEAN/Generate ROS Messages")]
        public static void Generate()
        {
            string rosPackage = Environment.GetEnvironmentVariable("SEAN_ROS_PACKAGE_DIR")
                ?? Path.GetFullPath("../social_sim_ros/social_sim_ros");
            string gizmo = Path.GetFullPath("Assets/Scripts/SEAN/Editor/MessageDefinitions/gizmo_msgs");

            foreach (var (dir, name) in new[] { (rosPackage, "social_sim_ros"), (gizmo, "gizmo_msgs") })
            {
                var warnings = MessageAutoGen.GeneratePackageMessages(dir, OutPath, name, true);
                foreach (string w in warnings) { UnityEngine.Debug.LogWarning(w); }
            }
            AssetDatabase.Refresh();
        }
    }
}
