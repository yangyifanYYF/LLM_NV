import matplotlib.pyplot as plt
import numpy as np

def visualize(start_point, all_obstacle_points, visible_obstacles, radius, cluster_labels=None, walkable_points=None):
    plt.figure(figsize=(8, 6))
    plt.scatter(start_point[0], start_point[1], color='red', label='Start Point')
    plt.scatter(all_obstacle_points[:, 0], all_obstacle_points[:, 1], s=100 * np.pi * radius ** 2, color='blue', alpha=0.5)
    plt.scatter([], [], s=100 * np.pi * radius ** 2, color='blue', alpha=0.5, label='Obstacles')

    if cluster_labels is None:
        plt.scatter(visible_obstacles[:, 0], visible_obstacles[:, 1], s=100 * np.pi * radius ** 2, color='green', alpha=0.5, label='Visible Obstacles')
    else:
        n_clusters = len(np.unique(cluster_labels))
        colors = plt.cm.tab10(np.linspace(0, 1, n_clusters))
        for i in range(n_clusters):
            cluster_visible_obstacles = visible_obstacles[cluster_labels == i]
            plt.scatter(cluster_visible_obstacles[:, 0], cluster_visible_obstacles[:, 1], s=100 * np.pi * radius ** 2, color=colors[i], alpha=0.5, label=f'Cluster {i+1}')

    if walkable_points is not None:
        plt.scatter(walkable_points[:, 0], walkable_points[:, 1], marker='x', label=f'Walkable Points')

    plt.legend(markerscale=0.1)
    plt.title('Visible Obstacles from Start Point')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
