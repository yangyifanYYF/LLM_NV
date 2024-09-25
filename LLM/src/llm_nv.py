import rospy
from nav_msgs.msg import OccupancyGrid
import ast
import numpy as np
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import MeanShift
from utils import find_visible_obstacles, find_walkable_points, cluster_and_average_coordinates, group_adjacent_coordinates, swap_rows, generate_goal_msgs
from ask_openai import ask
import time
import math
from pedsim_msgs.msg import TrackedPersons
from std_msgs.msg import Bool
from visualization import Visualizer
import datetime

class LLM_NV:
    def __init__(self):
        self.visualizer = Visualizer()
        rospy.sleep(1)
        rospy.Subscriber("robot_pose", PoseStamped, self.robot_pose_callback)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.costmap_callback)
        rospy.Subscriber("/pedsim_visualizer/tracked_persons", TrackedPersons, self.tracked_persons_callback)
        self.robot_position = None
        self.last_robot_position = None
        self.costmap_msg = None
        self.start_time = None
        self.reach_threshold = 0.1
        self.decision = None
        self.person_pose = []
        self.person_twist = []
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pause_publisher = rospy.Publisher('/pause_flag', Bool, queue_size=10)
        # 获取当前时间
        current_time = datetime.datetime.now()
        self.narrow = False
        self.no_obs = False
        self.time_cost =[]
        self.goal_reached = False

        # 格式化时间为字符串，例如：2024-05-09_135601
        time_string = current_time.strftime("%Y-%m-%d_%H%M%S")
        self.log_name = f"{time_string}.txt"
        
    
    def tracked_persons_callback(self, msg):
        self.person_pose = []
        self.person_twist = []
        for person in msg.tracks:
            self.person_pose .append([round(person.pose.pose.position.x,  1), round(person.pose.pose.position.y, 1)])
            self.person_twist.append([round(person.twist.twist.linear.x, 1), round(person.twist.twist.linear.y, 1)])

    def robot_pose_callback(self, msg):
        self.robot_position = [msg.pose.position.x, msg.pose.position.y]

    def costmap_callback(self, msg):
        self.costmap_msg = msg
    
    def llm(self, destination_point):
        if not self.costmap_msg:
            return
        if self.robot_position is None:
            return
        # 判断是否到达目的地点
        if self.goal_reached:
            if self.start_time is not None and math.sqrt((self.robot_position[0] - destination_point[0])**2 + (self.robot_position[1] - destination_point[1])**2)<self.reach_threshold:
                self.time_cost.append(time.time() - self.start_time)
                print('cost time:', self.time_cost)
                with open("/home/yyf/projects/LLM_NV/logs/"+self.log_name, "a") as file:
                    file.write(f"cost time: {sum(self.time_cost)}\n")
                # 关闭ros节点
                rospy.signal_shutdown("Reached destination")
                return
            else:
                return
        # 确保机器人已经到达目标点
        if self.start_time is not None and time.time() - self.start_time < 10 and math.sqrt((self.robot_position[0] - self.decision[0])**2 + (self.robot_position[1] - self.decision[1])**2)>self.reach_threshold:
            self.last_robot_position = self.robot_position
            return
        elif self.start_time is not None and (time.time() - self.start_time > 10 or math.sqrt((self.robot_position[0] - self.decision[0])**2 + (self.robot_position[1] - self.decision[1])**2)<self.reach_threshold):
            self.time_cost.append(time.time() - self.start_time)
        
        bool_msg = Bool()
        bool_msg.data = True  # 这里可以更改为 False，以发布不同的布尔值
        self.pause_publisher.publish(bool_msg)



        # 获取地图尺寸和数据
        width = self.costmap_msg.info.width
        height = self.costmap_msg.info.height
        print("地图尺寸：", width, height)
        # print("map timestamp: ", msg.header.stamp.secs)
        data = self.costmap_msg.data

        # 将数据转换为NumPy数组
        grid = np.array(data).reshape((height, width))
        swap_rows(grid)
        # print('gridtmp', grid.min(), grid.max(), np.sum(grid==grid.max()))


        # 获取机器人当前位置
        start_point = self.robot_position
        print ("机器人当前位置：", start_point)
        
        # 目标点位置可视化
        self.visualizer.publish_goal(destination_point)

        
        # 获取障碍物信息并进行处理
        visible_obstacles, original_visible_obstacles = group_adjacent_coordinates(find_visible_obstacles(grid, start_point), self.narrow)
        if self.narrow and self.no_obs==False:
            visible_obstacles = visible_obstacles[(visible_obstacles[:, 1] < 1.2) & (visible_obstacles[:, 1] > -1.2)]
        print("可见障碍物位置坐标：")
        print(visible_obstacles)
        if self.no_obs:
            visible_obstacles, original_visible_obstacles = np.array([[]]), [[]]

        if not self.no_obs:
            # 可见障碍物可视化
            self.visualizer.publish_visible_obstacles(original_visible_obstacles)
            # 仅保留横坐标与机器人当前位置的横坐标差值绝对值大于等于0.5的点
            if visible_obstacles.size != 0:
                visible_obstacles = visible_obstacles[visible_obstacles[:, 4] - start_point[0] >= 1.5]
            print("筛选后的可见障碍物位置坐标：")  
            print(visible_obstacles)

        if visible_obstacles.size == 0 and (self.person_pose is [] or self.robot_position[0]+0.5 >max(sublist[0] for sublist in self.person_pose)):
            bool_msg.data = False
            self.pause_publisher.publish(bool_msg)
            rospy.loginfo("Sending navigation goal...")
            self.goal_publisher.publish(generate_goal_msgs(destination_point, [destination_point[0]-1, destination_point[1]]))
            rospy.loginfo("Arrived at the destination point! Navigation ends!")
            self.start_time= time.time()
            self.goal_reached = True
            # rospy.signal_shutdown("Arrived at the destination point! Navigation ends!")

        # 如果生成的可见障碍物横坐标没有比行人横坐标小的，且机器人前方有行人，则可行走点为行人上下的两个点
        if self.person_pose is not [] and (visible_obstacles.size == 0 or np.min(visible_obstacles[:, 0]) > max(sublist[0] for sublist in self.person_pose)) and start_point[0] +0.5 < max(sublist[0] for sublist in self.person_pose):
            min_x = np.inf
            for pose in self.person_pose:  
                if pose[0] > start_point[0]+0.2 and pose[0] < min_x:
                    min_x = pose[0]
                    min_pose = pose
            walkable_points = [[min_pose[0], round(min_pose[1]-1.5, 1)], [min_pose[0], round(min_pose[1]+1.5, 1)]]
            print("可行走点坐标列表：")
            print(walkable_points)
            # 从文本文件中读取提示信息并替换变量
            # if self.no_obs:
            #     with open("./src/LLM_RL/src/prompts/obs_people_narrow.txt", "r") as file:
            #         prompt_content = file.read()
            # else:
            #     with open("./src/LLM_RL/src/prompts/obs_people.txt", "r") as file:
            #         prompt_content = file.read()
            with open("./src/LLM_RL/src/prompts/obs_people.txt", "r") as file:
                    prompt_content = file.read()

            with open("./src/LLM_RL/src/prompts/obs_people_role.txt", "r") as file:
                role_content = file.read()

            # 替换变量部分
            prompt_content_with_variables = prompt_content.format(
                current_location=[round(start_point[0], 1), round(start_point[1], 1)],
                visible_obstacles=visible_obstacles[:, :2] if visible_obstacles.size != 0 else [],
                destination=destination_point,
                guiding_points=walkable_points,
                pedestrian_position = self.person_pose,
                pedestrian_velocity = self.person_twist,
                scene_type = "a corridor" if self.narrow else "an open area",
                boundaries = [1.2, -1.2] if self.narrow else [7, -7]
            )
        else:
            # 使用Mean Shift聚类算法聚类可见障碍物的横坐标
            bandwidth = 1.2  # 手动设置带宽参数
            mean_shift = MeanShift(bandwidth=bandwidth)
            if visible_obstacles.size == 0:
                return
            cluster_labels = mean_shift.fit_predict(visible_obstacles[:, 0].reshape(-1, 1))

            print("Mean Shift聚类结果：")
            print(cluster_labels)

            # # 聚类后的障碍物可视化
            # self.visualizer.publish_clustered_obstacles(visible_obstacles, cluster_labels)

            # 获取可行走点坐标列表
            walkable_points = find_walkable_points(cluster_labels, visible_obstacles, grid, start_point)
            if self.narrow:
                for walkable_point in walkable_points:
                    if walkable_point[1] > 1.2:
                        walkable_point[1] = 1.2
                    elif walkable_point[1] < -1.2:
                        walkable_point[1] = -1.2
            print("可行走点坐标列表：")
            print(walkable_points)

            # 从文本文件中读取提示信息并替换变量
            with open("./src/LLM_RL/src/prompts/obs.txt", "r") as file:
                prompt_content = file.read()
            
            with open("./src/LLM_RL/src/prompts/obs_role.txt", "r") as file:
                role_content = file.read()

            with open("./src/LLM_RL/src/prompts/second_inf.txt", "r") as file:
                inf_content = file.read()

            # 替换变量部分
            prompt_content_with_variables = prompt_content.format(
                current_location=[round(start_point[0], 1), round(start_point[1], 1)],
                visible_obstacles=visible_obstacles[:, :2],
                destination=destination_point,
                guiding_points=walkable_points,
                scene_type = "a corridor" if self.narrow else "an open area",
                boundaries = [1.2, -1.2] if self.narrow else [7, -7]
            )


            # while self.decision not in walkable_points:
            #     original_decision = ask(role_content, prompt_content_with_variables).content
            #     inf_content_with_variables = inf_content.format(
            #         guiding_points=walkable_points,
            #         content=original_decision)
            #     inf_decision = ask('', inf_content_with_variables).content
            #     if inf_decision[0]=='[':
            #         self.decision =ast.literal_eval(inf_decision)
            #     with open("/home/yyf/projects/LLM_NV/logs/"+self.log_name, "a") as file:
            #         file.write(f"Role Content: {role_content}\n")
            #         file.write(f"Prompt Content with Variables: {prompt_content_with_variables}\n")
            #         file.write(f"Original Decision: {original_decision}\n")
            #         file.write(f"Inf Content with Variables: {inf_content_with_variables}\n")
            #         file.write(f"Decision: {self.decision}\n")
       
        # # ablation study
        # role_content = ''

        while self.decision not in walkable_points:    
            original_decision = ask(role_content, prompt_content_with_variables).content
            with open("/home/yyf/projects/LLM_NV/logs/"+self.log_name, "a") as file:
                file.write(f"Role Content: {role_content}\n")
                file.write(f"Prompt Content with Variables: {prompt_content_with_variables}\n")
                file.write(f"Original Decision: {original_decision}\n")
            import re
            # match = re.search(r"Therefore, the guiding point I choose is \[([\d\.,\-]+),\s*([\d\.,\-]+)\]", original_decision)
            # if match:
            #     x, y = match.groups()
            #     self.decision = [float(x), float(y)]
            if original_decision[0]=='[':
                self.decision = ast.literal_eval(original_decision)
            print("Decision:", self.decision)
        
        # 可行走点可视化
        self.visualizer.publish_walkable_points(self.robot_position, walkable_points, self.decision)

        bool_msg.data = False
        # 发布消息
        self.pause_publisher.publish(bool_msg)
        
        if self.person_pose is not [] and (visible_obstacles.size == 0 or np.min(visible_obstacles[:, 0]) > max(sublist[0] for sublist in self.person_pose)) and self.robot_position[0]+0.5 < max(sublist[0] for sublist in self.person_pose):
            rospy.loginfo("Sending navigation goal..."+str(self.decision))
            self.goal_publisher.publish(generate_goal_msgs(self.decision, self.robot_position))
            start_time = time.time()
            while math.sqrt((self.robot_position[0] - self.decision[0])**2 + (self.robot_position[1] - self.decision[1])**2)>1.0:
                if time.time() - start_time< 1:
                    continue
                min_x = np.inf
                for pose in self.person_pose:  
                    if pose[0] > self.robot_position[0]+0.2 and pose[0] < min_x:
                        min_x = pose[0]
                        min_pose = pose
                if self.decision[1] > min_pose[1]:
                    self.decision = [min_pose[0], min_pose[1] + 1.0]
                else:
                    self.decision = [min_pose[0], min_pose[1] - 1.0]
                rospy.loginfo("目标点修正："+str(self.decision))
                self.goal_publisher.publish(generate_goal_msgs(self.decision, self.robot_position))
                start_time = time.time()
            self.start_time = time.time()
            
        else:
            rospy.loginfo("Sending navigation goal..."+str(self.decision))
            self.goal_publisher.publish(generate_goal_msgs(self.decision, self.robot_position))
            self.start_time = time.time()