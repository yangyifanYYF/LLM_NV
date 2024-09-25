import numpy as np
import math
from sklearn.cluster import MeanShift
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

def cluster_and_average_coordinates(visible_obstacles):
    # Apply MeanShift clustering
    bandwidth = 1.2# 第一次聚类带宽
    ms = MeanShift(bandwidth=bandwidth)
    ms.fit(visible_obstacles)

    # Get cluster centers
    cluster_centers = ms.cluster_centers_

    # Create a dictionary to store cluster index and corresponding coordinates
    clusters = {}
    for i, cluster_center in enumerate(cluster_centers):
        clusters[i] = {'coordinates': [], 'center': cluster_center}

    # Assign each coordinate to its corresponding cluster
    for coordinate in visible_obstacles:
        cluster_index = ms.predict([coordinate])[0]
        clusters[cluster_index]['coordinates'].append(coordinate)

    # Calculate the mean coordinates and additional statistics for each cluster
    averaged_coordinates = []
    for cluster_index, cluster_data in clusters.items():
        cluster_mean = np.mean(cluster_data['coordinates'], axis=0)
        cluster_mean_rounded = np.round(cluster_mean, decimals=1)  # Round to one decimal place

        # Additional statistics
        coordinates_array = np.array(cluster_data['coordinates'])
        min_y = np.min(coordinates_array[:, 1])  # Minimum y-coordinate
        max_y = np.max(coordinates_array[:, 1])  # Maximum y-coordinate
        min_x = np.min(coordinates_array[:, 0])  # Minimum x-coordinate

        # Append mean coordinates and additional statistics to the result
        additional_stats = [min_y, max_y, min_x]
        result = np.concatenate((cluster_mean_rounded, additional_stats))
        averaged_coordinates.append(result)

    # Create original_visible_obstacles array with cluster labels
    original_visible_obstacles = np.zeros((visible_obstacles.shape[0], visible_obstacles.shape[1] + 1))
    original_visible_obstacles[:, :-1] = visible_obstacles
    original_visible_obstacles[:, -1] = ms.labels_

    return np.array(averaged_coordinates), original_visible_obstacles

def filter_groups(groups, y_min_threshold=-1.2, y_max_threshold=1.2):
    filtered_groups = []
    
    for group in groups:
        # 过滤当前分组中的点
        filtered_points = [
            point for point in group
            if y_min_threshold <= point[1] <= y_max_threshold
        ]
        
        # 如果过滤后的分组非空，则添加到结果列表中
        if filtered_points:
            filtered_groups.append(filtered_points)
    
    return filtered_groups


def group_adjacent_coordinates(visible_obstacles,  is_narrow):
    # 初始化分组列表
    distance_threshold = 0.1
    groups = []
    visited = np.zeros(len(visible_obstacles), dtype=bool)

    for i, point in enumerate(visible_obstacles):
        if visited[i]:
            continue
        
        # 新的分组
        current_group = [point]
        visited[i] = True
        stack = [point]

        # 使用栈进行深度优先搜索找到所有相邻点
        while stack:
            current_point = stack.pop()
            for j, other_point in enumerate(visible_obstacles):
                if not visited[j] and np.linalg.norm(current_point - other_point) <= distance_threshold:
                    current_group.append(other_point)
                    visited[j] = True
                    stack.append(other_point)

        groups.append(current_group)

    if is_narrow:
        # 过滤掉不符合条件的分组
        groups = filter_groups(groups)
    
    # 计算每个分组的平均坐标和其他统计数据
    averaged_coordinates = []
    for group in groups:
        group_array = np.array(group)
        group_mean = np.mean(group_array, axis=0)
        group_mean_rounded = np.round(group_mean, decimals=1)

        # 附加统计数据
        min_y = np.min(group_array[:, 1])
        max_y = np.max(group_array[:, 1])
        min_x = np.min(group_array[:, 0])

        additional_stats = [min_y, max_y, min_x]
        result = np.concatenate((group_mean_rounded, additional_stats))
        averaged_coordinates.append(result)

    
    return np.array(averaged_coordinates), groups


# def group_adjacent_coordinates(obstacles, is_narrow):
    visible_obstacles, local_visible_obstacles = obstacles
    # 初始化分组列表
    distance_threshold=1
    groups = []
    visited = np.zeros(len(local_visible_obstacles), dtype=bool)

    for i, point in enumerate(local_visible_obstacles):
        if visited[i]:
            continue
        
        # 新的分组
        current_group = [point]
        visited[i] = True

        # 找到与当前点相邻的所有点
        for j, other_point in enumerate(local_visible_obstacles):
            if not visited[j] and np.linalg.norm(point - other_point) <= distance_threshold:
                current_group.append(visible_obstacles[j])
                visited[j] = True

        groups.append(current_group)

    # 计算每个分组的平均坐标和其他统计数据
    averaged_coordinates = []
    for group in groups:
        group_array = np.array(group)
        group_mean = np.mean(group_array, axis=0)
        group_mean_rounded = np.round(group_mean, decimals=1)

        # 附加统计数据
        min_y = np.min(group_array[:, 1])
        max_y = np.max(group_array[:, 1])
        min_x = np.min(group_array[:, 0])

        additional_stats = [min_y, max_y, min_x]
        result = np.concatenate((group_mean_rounded, additional_stats))
        averaged_coordinates.append(result)

    if is_narrow:
        # 过滤掉不符合条件的分组
        groups = filter_groups(groups)
    return np.array(averaged_coordinates), groups


def find_visible_obstacles(grid, robot_position):
    visible_obstacles = set()  # 使用集合来确保元素的唯一性\
    # local_visible_obstacles = set()

    # Define grid dimensions
    rows, cols = grid.shape

    # Define robot position
    robot_x, robot_y = robot_position

    # Iterate through each degree (360 degrees)
    for degree in range(-90, 90):
        # Convert degree to radians
        angle_rad = math.radians(degree)

        # 机器人当前位置一定在局部地图中心
        current_row, current_col = rows // 2, cols // 2

        # Iterate until the edge of the grid is reached or an obstacle is hit
        while 0 <= current_row < rows and 0 <= current_col < cols:
            # Move in the direction of the current angle
            current_row += math.sin(angle_rad)
            current_col += math.cos(angle_rad)

            # Round current position to nearest integer to get grid coordinates
            row_index = round(current_row)
            col_index = round(current_col)

            # Check if the current position is within the grid and if there is an obstacle
            if 0 <= row_index < rows and 0 <= col_index < cols and grid[row_index, col_index] != 0:
                # Mark the obstacle as visible and stop exploring in this direction
                # print("障碍物行列索引：", row_index, col_index)
                # 把行列索引转换回地图坐标
                x = robot_x-(cols//2-col_index) * 5 / (cols//2)
                y = robot_y+(rows//2-row_index) * 5 / (rows//2)
                # print("障碍物坐标：", x, y)
                visible_obstacles.add((x, y))  # 把坐标添加到集合中
                # local_visible_obstacles.add((row_index, col_index))
            if 0 <= row_index < rows and 0 <= col_index < cols and grid[row_index, col_index] == 100:
                break

    # Convert the set of tuples to a NumPy array
    return np.array(list(visible_obstacles))

def is_obstacle(grid, x, y, robot_position):
    rows, cols = grid.shape
    robot_x, robot_y = robot_position
    col_index = int((x - robot_x) * (cols // 2) / 5 + (cols // 2))
    row_index = int((y - robot_y) * (rows // 2) / 5 + (rows // 2))
    print("行列索引：", row_index, col_index)
    if grid[row_index][col_index] != 0:
        return True
    else:
        return False


def find_walkable_points(cluster_labels, visible_obstacles, grid, robot_position):
    # print(is_obstacle(grid,2.4,1.5, robot_position))
    walkable_points = []
    min_avg_x = float('inf')
    min_avg_label = None

    # 找到横坐标平均值最小的聚类标签
    for cluster_label in np.unique(cluster_labels):
        cluster_visible_obstacles = visible_obstacles[cluster_labels == cluster_label]
        avg_x = np.mean(cluster_visible_obstacles[:, 0])
        if avg_x < min_avg_x:
            min_avg_x = avg_x
            min_avg_label = cluster_label

    # 找到横坐标平均值最小的一类障碍物的可行走点
    cluster_visible_obstacles = visible_obstacles[cluster_labels == min_avg_label]
    sorted_cluster_obstacles = cluster_visible_obstacles[np.argsort(cluster_visible_obstacles[:, 1])]

    # 添加纵坐标最小的障碍物下方一个单位的点
    downside_point = [round(sorted_cluster_obstacles[0][4]-0.3, 1), round(sorted_cluster_obstacles[0][2] - 0.2, 1)]
    # if is_obstacle(grid, downside_point[0], downside_point[1], robot_position):
    #     walkable_points.append([sorted_cluster_obstacles[0][0], sorted_cluster_obstacles[0][1] - 2])
    # else:
    walkable_points.append(downside_point)

    for i in range(len(sorted_cluster_obstacles) - 1):
        # midpoint = (sorted_cluster_obstacles[i][0:2] + sorted_cluster_obstacles[i + 1][0:2]) / 2
        midpoint = [(sorted_cluster_obstacles[i][0] + sorted_cluster_obstacles[i + 1][0]) / 2, (sorted_cluster_obstacles[i][3] + sorted_cluster_obstacles[i + 1][2]) / 2]
        # if not is_obstacle(grid, midpoint[0], midpoint[1], robot_position):
        walkable_points.append([round(midpoint[0]-0.1, 1), round(midpoint[1], 1)])

    # 添加纵坐标最大的障碍物上方一个单位的点
    upside_point = [round(sorted_cluster_obstacles[-1][4]-0.3, 1), round(sorted_cluster_obstacles[-1][3] + 0.2, 1)]
    # if is_obstacle(grid, upside_point[0], upside_point[1], robot_position):
    #     walkable_points.append([sorted_cluster_obstacles[-1][0], sorted_cluster_obstacles[-1][1] + 2])
    # else:
    walkable_points.append(upside_point)


    return walkable_points

def line_equation(p1, p2):
    # Calculate coefficients of the line equation ax + by + c = 0 passing through points p1 and p2
    x1, y1 = p1
    x2, y2 = p2
    a = y2 - y1
    b = x1 - x2
    c = x2 * y1 - x1 * y2
    return a, b, c

def distance_to_line(a, b, c, x, y):
    # Calculate the distance from point (x, y) to the line ax + by + c = 0
    distance = abs(a * x + b * y + c) / np.sqrt(a**2 + b**2)
    return distance

def swap_rows(grid):
    n_rows = grid.shape[0]
    for i in range(n_rows // 2):
        grid[i], grid[n_rows - i - 1] = grid[n_rows - i - 1].copy(), grid[i].copy()

def generate_goal_msgs(decision, start_point):
    # 四元数为机器人当前位置到目标位置的射线相对于x轴正方向的旋转
    ## 计算从机器人当前位置到目标位置的向量
    direction_vector = np.array(decision) - np.array(start_point)

    ## 计算旋转角度
    rotation_angle = np.arctan2(direction_vector[1], direction_vector[0])

    ## 使用NumPy进行欧拉角到四元数的转换
    quaternion_np = Rotation.from_euler('zyx', np.array([rotation_angle,0,0])).as_quat()

    # 发布导航目标
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"  # 设置坐标系为地图坐标系
    goal_msg.pose.position.x = decision[0]  # 目标点的x坐标
    goal_msg.pose.position.y = decision[1]  # 目标点的y坐标
    # goal_msg.pose.orientation.w = 1.0  # 设置四元数表示的方向，这里设为单位四元数，表示无旋转
    # 将四元数表示的旋转赋值到导航目标的方向
    goal_msg.pose.orientation.x = quaternion_np[0]
    goal_msg.pose.orientation.y = quaternion_np[1]
    goal_msg.pose.orientation.z = quaternion_np[2]
    goal_msg.pose.orientation.w = quaternion_np[3]
    return goal_msg
