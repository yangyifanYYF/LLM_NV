import rospy
from llm_nv import LLM_NV

if __name__ == '__main__':
    rospy.init_node('costmap_listener', anonymous=True)
    llm_nv = LLM_NV()
    rospy.sleep(1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        llm_nv.llm([11,0])
        # llm_nv.llm([8, 0])
        rate.sleep()
    
