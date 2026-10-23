using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.UI;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;


public class DisplaySocialSituation : MonoBehaviour
{
    public Text text;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<RosMessageTypes.Std.StringMsg>("lifecycle_learner/social_situation", ReceiveMessage);
    }

    void ReceiveMessage(RosMessageTypes.Std.StringMsg message)
    {
        text.text = "Social situation: " + message.data;
    }
}
