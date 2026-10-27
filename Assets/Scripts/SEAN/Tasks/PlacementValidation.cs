using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

namespace SEAN.Tasks
{
    // Opt-in audit of the running scene; never changes the active task or RNG state.
    public class PlacementValidation : MonoBehaviour
    {
        [Serializable] public class Pair { public int seed; public Vector3 start, goal; public bool found; }
        [Serializable] public class Audit
        {
            public bool knownBadStartClear;
            public string[] knownBadColliders;
            public Pair[] pairs;
        }
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install()
        {
            if (!System.Environment.GetCommandLineArgs().Contains("-placement-validation-output")) return;
            // The initial LoadingScene is replaced before the robot is available.
            if (FindObjectOfType<PlacementValidation>() != null) return;
            var go = new GameObject("Placement validation");
            DontDestroyOnLoad(go);
            go.AddComponent<PlacementValidation>();
        }
        private IEnumerator Start()
        {
            while (SEAN.instance == null || SEAN.instance.robot == null) yield return null;
            yield return new WaitForSeconds(8);
            string[] args = System.Environment.GetCommandLineArgs();
            string output = args[Array.IndexOf(args, "-placement-validation-output") + 1];
            Transform robot = SEAN.instance.robot.gameObject.transform;
            var state = UnityEngine.Random.state;
            var records = new List<Pair>();
            Audit audit;
            try
            {
                var bad = new Vector3(7.7116866f, 0.051449f, 1.6128832f);
                Physics.SyncTransforms();
                audit = new Audit {
                    knownBadStartClear = RandomTaskPlacement.IsClear(bad, robot),
                    knownBadColliders = Physics.OverlapBox(bad + Vector3.up * 1.9f,
                        new Vector3(0.65f, 1.8f, 0.65f), Quaternion.identity, ~0, QueryTriggerInteraction.Ignore)
                        .Select(c => c.name).ToArray()
                };
                foreach (int seed in new[] { 0, 1, 7, 42, 12345 })
                {
                    UnityEngine.Random.InitState(seed);
                    for (int i = 0; i < 20; i++)
                    {
                        bool found = RandomTaskPlacement.TryPair(robot, out Vector3 start, out Vector3 goal);
                        records.Add(new Pair { seed = seed, found = found, start = start, goal = goal });
                    }
                }
                audit.pairs = records.ToArray();
            }
            finally { UnityEngine.Random.state = state; }
            File.WriteAllText(output, JsonUtility.ToJson(audit, true));
            Debug.Log("Placement validation audit written: " + output);
        }
    }
}
