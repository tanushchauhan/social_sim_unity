using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public static class RosPublishExtensions
{
    public static void PublishAuto<T>(this ROSConnection ros, string topic, T message) where T : Message
    {
        var state = ros.GetTopic(topic);
        if (state == null || !state.IsPublisher)
        {
            ros.RegisterPublisher<T>(topic);
        }
        ros.Publish(topic, message);
    }
}
