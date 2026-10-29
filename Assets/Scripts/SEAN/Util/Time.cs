namespace SEAN.Util
{
    public class Time
    {
        public static double Milliseconds(double startMillis = 0, double time = 0)
        {
            if (time == 0)
            {
                time = UnityEngine.Time.time;
            }
            // Time.time is in seconds -> *1000 -> milliseconds
            double msecs = startMillis + 1000 * time;
            //uint sec = (uint)(msecs / 1000);
            //uint nanosec = (uint)((msecs / 1000 - sec) * 1e+9);
            //print("msecs: " + sec + "." + nanosec);
            return msecs;
        }

        public static RosMessageTypes.BuiltinInterfaces.TimeMsg MTime(double milliseconds)
        {
            int sec = (int)(milliseconds / 1000);
            return new RosMessageTypes.BuiltinInterfaces.TimeMsg
            {
                sec = sec,
                nanosec = (uint)((milliseconds / 1000 - sec) * 1e+9)
            };
        }

    }
}
