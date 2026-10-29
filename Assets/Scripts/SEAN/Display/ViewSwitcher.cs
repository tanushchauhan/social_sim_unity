using System;
using UnityEngine;

namespace SEAN.Display
{
    /// <summary>
    /// Chooses the on-screen view with the C key or the -view argument.
    /// Auto keeps the task's own layout. Other views draw one existing camera
    /// full screen above the rest. ROS camera topics render through separate
    /// offscreen copies, so switching does not change published images.
    /// </summary>
    public class ViewSwitcher : MonoBehaviour
    {
        public enum View { Auto, FirstPerson, ThirdPerson, Overhead, Top }

        public KeyCode key = KeyCode.C;
        public View view { get; private set; } = View.Auto;

        private const float OverlayDepth = 100f;
        private static readonly Rect FullScreen = new Rect(0f, 0f, 1f, 1f);

        private SEAN sean;
        private Camera selected;
        private Rect savedRect;
        private float savedDepth;
        private bool savedEnabled;
        private ushort savedTask;

        public static View Next(View current)
        {
            return (View)(((int)current + 1) % Enum.GetValues(typeof(View)).Length);
        }

        public static View Parse(string name)
        {
            foreach (View candidate in Enum.GetValues(typeof(View)))
            {
                if (string.Equals(candidate.ToString(), name.Replace("-", ""), StringComparison.OrdinalIgnoreCase))
                {
                    return candidate;
                }
            }
            throw new ArgumentException("Unknown view " + name + ", valid options are " + string.Join(", ", Enum.GetNames(typeof(View))));
        }

        private void Start()
        {
            sean = SEAN.instance;
            // Participant sessions keep the player's own camera.
            if (sean.PlayerControl)
            {
                enabled = false;
                return;
            }
            string[] args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < args.Length; i++)
            {
                if (args[i] == "-view") { Select(Parse(args[i + 1])); }
            }
        }

        private void Update()
        {
            if (UnityEngine.Input.GetKeyDown(key)) { Select(Next(view)); }
        }

        // Runs after task code, which re-enables the first-person camera at
        // each new task, so the chosen view stays on top.
        private void LateUpdate()
        {
            if (selected == null) { return; }
            selected.enabled = true;
            selected.rect = FullScreen;
            selected.depth = OverlayDepth;
        }

        public void Select(View next)
        {
            Restore();
            view = next;
            selected = CameraFor(next);
            if (selected != null)
            {
                savedRect = selected.rect;
                savedDepth = selected.depth;
                savedEnabled = selected.enabled;
                savedTask = sean.robotTask.number;
            }
            Debug.Log("SEAN view: " + view);
        }

        private void Restore()
        {
            if (selected == null) { return; }
            selected.rect = savedRect;
            selected.depth = savedDepth;
            // A task that started meanwhile hid the top view; keep that outcome.
            bool topHiddenByTask = selected == sean.environment.topViewCamera && sean.robotTask.number != savedTask;
            selected.enabled = topHiddenByTask ? sean.TopDownViewOnly : savedEnabled;
            selected = null;
        }

        private Camera CameraFor(View candidate)
        {
            switch (candidate)
            {
                case View.FirstPerson: return sean.robot.camera_first;
                case View.ThirdPerson: return sean.robot.camera_third;
                case View.Overhead: return sean.robot.camera_overhead;
                case View.Top: return sean.environment.topViewCamera;
                default: return null;
            }
        }

        private void OnGUI()
        {
            int w = Screen.width, h = Screen.height;
            GUIStyle style = new GUIStyle();
            style.alignment = TextAnchor.LowerRight;
            style.fontSize = h * 2 / 100;
            style.normal.textColor = new Color(0.75f, 0.75f, 0.75f, 1.0f);
            GUI.Label(new Rect(0, h - h * 3 / 100, w - 10, h * 2 / 100), "View: " + view + " (" + key + " to switch)", style);
        }
    }
}
