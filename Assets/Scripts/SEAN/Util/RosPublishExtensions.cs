using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

/// <summary>
/// ROS-TCP-Connector 0.7 throws when publishing on a topic that was never registered.
/// PublishAuto registers the publisher on first use, then publishes.
/// </summary>
public static class RosPublishExtensions
{
    static readonly HashSet<string> registered = new HashSet<string>();

    public static void PublishAuto<T>(this ROSConnection ros, string topic, T message) where T : Message
    {
        if (registered.Add(topic))
        {
            ros.RegisterPublisher<T>(topic);
        }
        ros.Publish(topic, message);
    }
}
