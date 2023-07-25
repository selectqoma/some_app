import time
import rospy

def calculate_fps(timestamps):
    if len(timestamps) == 0:
        return 0

    dt = (rospy.Time.now() - timestamps[0]).to_sec()
    return len(timestamps) / dt
