import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal():
    rospy.init_node('goal_publisher', anonymous=True)
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(1) # 发布速率为1Hz

    while not rospy.is_shutdown():
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  # 假设目标在map坐标系下
        goal_msg.pose.position.x =  -5   # 假设目标x坐标为1.0
        goal_msg.pose.position.y =0   # 假设目标y坐标为2.0
        goal_msg.pose.orientation.w = 1.0  # 假设目标朝向为正方向

        goal_publisher.publish(goal_msg)
        rospy.loginfo("Published goal: x={}, y={}".format(goal_msg.pose.position.x, goal_msg.pose.position.y))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass

