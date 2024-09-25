import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import random  # 如果要生成随机颜色的话，需要导入random库
from math import sqrt

class Visualizer:
    def __init__(self):
        self.visible_obstacles_publisher = rospy.Publisher('/visible_obstacles', Marker, queue_size=10)
        self.clustered_obstacles_publisher = rospy.Publisher('/clustered_obstacles', MarkerArray, queue_size=10)
        self.walkable_points_publisher = rospy.Publisher('/walkable_points', MarkerArray, queue_size=1)
        self.goal_publisher = rospy.Publisher('/destination_point', Marker, queue_size=10)

    def publish_visible_obstacles(self, original_visible_obstacles):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0  # 完全不透明

        for lst in original_visible_obstacles:
             # 随机生成颜色
            color = (random.random(), random.random(), random.random())
            for point in lst:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = 0
                marker.points.append(p)

                # 设置颜色
                marker.colors.append(ColorRGBA(color[0], color[1], color[2], 1.0))

        self.visible_obstacles_publisher.publish(marker)

    def publish_clustered_obstacles(self, visible_obstacles, cluster_labels):
        # 创建一个 MarkerArray 用于存储多个 Marker
        marker_array = MarkerArray()

        # 创建每个类别的障碍物的 Marker
        unique_labels = set(cluster_labels)
        for cluster_label in unique_labels:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "clustered_obstacles"
            marker.id = cluster_label
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.color.a = 1.0
            
            # 根据类别设置不同的颜色
            marker.color.r, marker.color.g, marker.color.b = self.get_color_for_label(cluster_label)

            # 将属于当前类别的障碍物添加到对应的 Marker 中
            for i, label in enumerate(cluster_labels):
                if label == cluster_label:
                    p = Point()
                    p.x = visible_obstacles[i][0]
                    p.y = visible_obstacles[i][1]
                    p.z = 0
                    marker.points.append(p)

            # 将创建的 Marker 添加到 MarkerArray 中
            marker_array.markers.append(marker)

        # 发布 MarkerArray
        self.clustered_obstacles_publisher.publish(marker_array)

    def get_color_for_label(self, label):
        # 根据类别返回不同的颜色
        # 这里可以根据需要自定义不同类别的颜色
        if label == 0:
            return 1.0, 0.0, 0.0  # 红色
        elif label == 1:
            return 0.0, 1.0, 0.0  # 绿色
        elif label == 2:
            return 0.0, 0.0, 1.0  # 蓝色
        else:
            return 0.5, 0.5, 0.5  # 灰色
        
    def publish_walkable_points(self, robot_position, walkable_points, decision):
        marker_array = MarkerArray()
         # 为每个可行走点创建一个箭头
        for idx, walkable_point in enumerate(walkable_points):
            # 创建一个新的Marker消息
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "walkable_points"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # 箭头宽度
            marker.scale.y = 0.2  # 箭头长度
            marker.color.a = 1.0  # 不透明度

            # 如果当前点是决策点，使用紫色，否则使用绿色
            if walkable_point == decision:
                marker.color.r = 0.0  # 红色分量
                marker.color.g = 1.0  # 绿色分量
                marker.color.b = 0.0  # 蓝色分量
            else:
                marker.color.r = 1.0  # 红色分量
                marker.color.g = 0.0  # 绿色分量
                marker.color.b = 0.0  # 蓝色分量

       
            # 计算方向向量
            direction_vector = [walkable_point[0] - robot_position[0],
                                walkable_point[1] - robot_position[1]]
            # 计算方向向量的长度
            length = sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
            # 归一化方向向量
            direction_vector = [direction_vector[0] / length, direction_vector[1] / length]
            # 设置箭头的起点
            start_point = Point()
            start_point.x = walkable_point[0] - direction_vector[0]
            start_point.y = walkable_point[1] - direction_vector[1]
            start_point.z = 0
            marker.points.append(start_point)
            # 设置箭头的终点
            end_point = Point()
            end_point.x = walkable_point[0]
            end_point.y = walkable_point[1]
            end_point.z = 0
            marker.points.append(end_point)
            # 将Marker添加到MarkerArray中
            marker_array.markers.append(marker)

        # 发布Marker消息
        self.walkable_points_publisher.publish(marker_array)

    def publish_goal(self, goal):
        # 创建一个新的Marker消息
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 箭头宽度
        marker.scale.y = 0.2  # 箭头长度
        marker.color.a = 1.0  # 不透明度
        marker.color.r = 1.0  # 红色分量
        marker.color.g = 0.0  # 绿色分量
        marker.color.b = 1.0  # 蓝色分量

    
        # 计算方向向量
        direction_vector = [1,0]
        # 计算方向向量的长度
        length = sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        # 归一化方向向量
        direction_vector = [direction_vector[0] / length, direction_vector[1] / length]
        # 设置箭头的起点
        start_point = Point()
        start_point.x = goal[0] - direction_vector[0]
        start_point.y = goal[1] - direction_vector[1]
        start_point.z = 0
        marker.points.append(start_point)
        # 设置箭头的终点
        end_point = Point()
        end_point.x = goal[0]
        end_point.y = goal[1]
        end_point.z = 0
        marker.points.append(end_point)

        # 发布Marker消息
        self.goal_publisher.publish(marker)

