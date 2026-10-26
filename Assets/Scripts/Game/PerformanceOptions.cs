using UnityEngine;

public class PerformanceOptions : MonoBehaviour
{
    public int TargetFrameRate = 60;

    private void Awake()
    {
#if !UNITY_EDITOR
        string[] args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (args[i] == "-enablelogs")
                Debug.unityLogger.logEnabled = args[++i] == "1";
            else if (args[i] == "-frame-rate")
                TargetFrameRate = int.Parse(args[++i]);
        }
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = Mathf.Max(1, TargetFrameRate);
#endif
    }
}
