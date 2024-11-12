#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def mock_sensor():
    rospy.init_node('mock_sensor', anonymous=True)
    pub = rospy.Publisher('/stair_detection', Float32, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    # Define pattern of values
    values = [10, 10, 10, 60, 10, 10, 10]
    
    # Publish each value once
    for value in values:
        if rospy.is_shutdown():
            break
            
        # Create and publish message
        msg = Float32()
        msg.data = value
        pub.publish(msg)
        
        rospy.loginfo(f"Published value: {value}")
        rate.sleep()
    
    # Shutdown after completing one loop
    rospy.loginfo("Completed one loop, shutting down...")
    rospy.signal_shutdown("Completed pattern")

if __name__ == '__main__':
    try:
        mock_sensor()
    except rospy.ROSInterruptException:
        pass
