import rospy
import geometry_msgs.msg


def movingthechair():
    rospy.init_node('chairmover', anonymous=True)
    pub = rospy.Publisher("tb3_1/cmd_vel", geometry_msgs.msg.Twist, queue_size= 50)
    vel = geometry_msgs.msg.Twist()
    rate = rospy.Rate(50)
 
    while True:
        for t in range(-5, 5):
            vel = geometry_msgs.msg.Twist()
            vel.linear.x = t/10
            print(vel.linear.x)
            pub.publish(vel)
            rate.sleep()

        for t in range(5, -5):
            vel = geometry_msgs.msg.Twist()
            vel.linear.x = t/10
            # print(vel.linear.x)
            pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        movingthechair()
    except rospy.ROSInterruptException:
        pass
