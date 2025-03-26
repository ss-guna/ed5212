#!/usr/bin/env python3

import rospy
import numpy as np
import random
import time
from gazebo_msgs.msg import ModelStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf.transformations import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Obstacle:
    def __init__(self, name, position, size, shape='cube'):
        self.name = name
        self.position = np.array(position)
        self.size = size
        self.shape = shape
        
    def check_collision(self, point):
        if self.shape == 'cube':
            return all(np.abs(point - self.position) <= self.size/2)
        elif self.shape == 'sphere':
            return np.linalg.norm(point - self.position) <= self.size/2
        return False

class KUKAiiwaRRTPlanner:
    def __init__(self):
        rospy.init_node('kuka_iiwa_rrt_planner')
        
        # ROS interfaces
        self.joint_pub = rospy.Publisher(
            '/iiwa/PositionJointInterface_trajectory_controller/command',
            JointTrajectory,
            queue_size=10
        )
        self.joint_sub = rospy.Subscriber(
            '/iiwa/joint_states', 
            JointState, 
            self.joint_state_callback
        )
        self.models_sub = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.models_callback
        )
        
        # Joint information
        self.joint_names = [
            'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',
            'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6',
            'iiwa_joint_7'
        ]
        self.joint_limits = np.array([
            [-2.96, 2.96], [-2.09, 2.09], [-2.96, 2.96],
            [-2.09, 2.09], [-2.96, 2.96], [-2.09, 2.09],
            [-3.05, 3.05]
        ])
        
        # Robot dimensions (approximate)
        self.link_lengths = [0.34, 0.0, 0.4, 0.0, 0.4, 0.0, 0.126]
        
        # Environment information
        self.table_height = 0.75
        self.table_size = 1.0
        self.obstacles = []
        self.models_received = False
        
        # Planning parameters
        self.max_step = 0.2
        self.max_iter = 5000
        self.goal_bias = 0.1
        
        # Visualization
        self.fig = plt.figure(figsize=(12, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Current state
        self.current_joints = None
        self.ready = False
        
        # Wait for initial data
        rospy.loginfo("Waiting for initial data...")
        while not rospy.is_shutdown() and (not self.ready or not self.models_received):
            rospy.sleep(0.1)
        
        self.setup_visualization()
        rospy.loginfo("KUKA iiwa RRT Planner ready!")

    def models_callback(self, msg):
        """Process Gazebo model states to extract obstacles"""
        if not self.models_received:
            self.obstacles = []
            
            for i, name in enumerate(msg.name):
                if name == 'ground_plane' or name == 'iiwa':
                    continue
                    
                pose = msg.pose[i]
                position = [pose.position.x, pose.position.y, pose.position.z]
                
                # Simple assumption about object sizes
                if 'table' in name.lower():
                    size = 1.0  # Assuming standard table size
                    self.obstacles.append(Obstacle(name, position, size, 'cube'))
                    self.table_height = pose.position.z + size/2
                elif 'box' in name.lower():
                    size = 0.2  # Assuming small box
                    self.obstacles.append(Obstacle(name, position, size, 'cube'))
                elif 'sphere' in name.lower():
                    size = 0.15  # Assuming small sphere
                    self.obstacles.append(Obstacle(name, position, size, 'sphere'))
            
            self.models_received = True
            rospy.loginfo(f"Detected {len(self.obstacles)} obstacles in environment")

    def joint_state_callback(self, msg):
        """Store current joint positions"""
        if not self.ready:
            self.current_joints = np.zeros(len(self.joint_names))
            for i, name in enumerate(self.joint_names):
                try:
                    idx = msg.name.index(name)
                    self.current_joints[i] = msg.position[idx]
                except ValueError:
                    rospy.logwarn(f"Joint {name} not found in joint_states")
            
            if all(p is not None for p in self.current_joints):
                self.ready = True
                rospy.loginfo(f"Initial joints: {self.current_joints}")

    def setup_visualization(self):
        """Initialize 3D visualization with all obstacles"""
        self.ax.clear()
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_zlim(0, 2.0)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('KUKA iiwa RRT Planning with Environment')
        
        # Draw all obstacles
        for obs in self.obstacles:
            if obs.shape == 'cube':
                self.plot_cube(obs.position, obs.size, obs.name)
            elif obs.shape == 'sphere':
                self.plot_sphere(obs.position, obs.size, obs.name)
        
        plt.draw()
        plt.pause(0.1)

    def plot_cube(self, center, size, name):
        """Plot a cube obstacle"""
        x, y, z = center
        s = size/2
        vertices = [
            [x-s, y-s, z-s], [x+s, y-s, z-s], [x+s, y+s, z-s], [x-s, y+s, z-s],
            [x-s, y-s, z+s], [x+s, y-s, z+s], [x+s, y+s, z+s], [x-s, y+s, z+s]
        ]
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[4], vertices[5], vertices[6], vertices[7]], 
            [vertices[0], vertices[1], vertices[5], vertices[4]], 
            [vertices[2], vertices[3], vertices[7], vertices[6]], 
            [vertices[1], vertices[2], vertices[6], vertices[5]], 
            [vertices[0], vertices[3], vertices[7], vertices[4]]
        ]
        cube = Poly3DCollection(faces, alpha=0.5, linewidths=1, edgecolor='r', facecolor='red')
        self.ax.add_collection3d(cube)
        self.ax.text(x, y, z+s+0.05, name, color='black')

    def plot_sphere(self, center, radius, name):
        """Plot a sphere obstacle"""
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        self.ax.plot_surface(x, y, z, color='blue', alpha=0.5)
        self.ax.text(center[0], center[1], center[2]+radius+0.05, name, color='black')

    def forward_kinematics(self, joints):
        """Simple forward kinematics for KUKA iiwa"""
        x = 0
        y = 0
        z = self.table_height  # Start from table surface
        
        # Calculate end effector position
        x += np.sin(joints[0]) * (
            self.link_lengths[2] * np.cos(joints[1]) + 
            self.link_lengths[4] * np.cos(joints[1] + joints[3]) +
            self.link_lengths[6] * np.cos(joints[1] + joints[3] + joints[5])
        )
        
        y += np.cos(joints[0]) * (
            self.link_lengths[2] * np.cos(joints[1]) +
            self.link_lengths[4] * np.cos(joints[1] + joints[3]) +
            self.link_lengths[6] * np.cos(joints[1] + joints[3] + joints[5])
        )
        
        z += (
            self.link_lengths[2] * np.sin(joints[1]) +
            self.link_lengths[4] * np.sin(joints[1] + joints[3]) +
            self.link_lengths[6] * np.sin(joints[1] + joints[3] + joints[5])
        )
        
        return np.array([x, y, z])

    def is_config_valid(self, config):
        """Check if configuration is valid (within limits and collision-free)"""
        # Check joint limits
        for i, q in enumerate(config):
            if not (self.joint_limits[i][0] <= q <= self.joint_limits[i][1]):
                return False
                
        # Check for collisions with all obstacles
        ee_pos = self.forward_kinematics(config)
        for obs in self.obstacles:
            if obs.check_collision(ee_pos):
                return False
                
        # Ensure end effector stays above table
        if ee_pos[2] < self.table_height + 0.05:  # 5cm safety margin
            return False
            
        return True

    def random_config(self):
        """Generate random configuration within joint limits"""
        return np.array([random.uniform(lim[0], lim[1]) for lim in self.joint_limits])

    def nearest_neighbor(self, nodes, target):
        """Find nearest node to target configuration"""
        if not nodes:
            return None
        distances = [np.linalg.norm(node.config - target) for node in nodes]
        return nodes[np.argmin(distances)]

    def steer(self, from_node, to_config):
        """Move from node toward configuration"""
        direction = to_config - from_node.config
        distance = np.linalg.norm(direction)
        if distance <= self.max_step:
            return to_config
        return from_node.config + (direction / distance) * self.max_step

    def rrt_plan(self, q_start, q_goal):
        """RRT planning algorithm"""
        nodes = [Node(q_start)]
        
        for iteration in range(self.max_iter):
            # Sample with goal bias
            if random.random() < self.goal_bias:
                rand_config = q_goal
            else:
                rand_config = self.random_config()
                while not self.is_config_valid(rand_config):
                    rand_config = self.random_config()
            
            # Find nearest node
            nearest = self.nearest_neighbor(nodes, rand_config)
            if nearest is None:
                continue
                
            # Steer toward sample
            new_config = self.steer(nearest, rand_config)
            
            # Check if path is valid
            if self.is_config_valid(new_config):
                new_node = Node(new_config, nearest)
                nodes.append(new_node)
                
                # Visualize new node
                ee_pos = self.forward_kinematics(new_config)
                self.ax.scatter(ee_pos[0], ee_pos[1], ee_pos[2], c='b', s=5, alpha=0.1)
                plt.pause(0.001)
                
                # Check if reached goal
                if np.linalg.norm(new_config - q_goal) < 0.2:  # Goal threshold
                    rospy.loginfo(f"Goal reached after {iteration} iterations")
                    return self.build_path(new_node)
        
        rospy.logwarn("Max iterations reached without finding path")
        return None

    def build_path(self, goal_node):
        """Reconstruct path from goal to start"""
        path = []
        current = goal_node
        while current is not None:
            path.append(current.config)
            current = current.parent
        return path[::-1]

    def execute_trajectory(self, path):
        """Execute planned path on robot"""
        if not path:
            rospy.logerr("Empty path - cannot execute")
            return False
            
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.joint_names
        
        duration = 0.0
        time_step = 1.0  # Time between points
        
        for i, config in enumerate(path):
            point = JointTrajectoryPoint()
            point.positions = config
            point.time_from_start = rospy.Duration(duration)
            traj.points.append(point)
            
            # Visualize path
            ee_pos = self.forward_kinematics(config)
            self.ax.scatter(ee_pos[0], ee_pos[1], ee_pos[2], c='g', s=20)
            if i > 0:
                prev_pos = self.forward_kinematics(path[i-1])
                self.ax.plot(
                    [prev_pos[0], ee_pos[0]], 
                    [prev_pos[1], ee_pos[1]], 
                    [prev_pos[2], ee_pos[2]], 
                    'g-'
                )
            plt.pause(0.01)
            
            duration += time_step
        
        self.joint_pub.publish(traj)
        rospy.loginfo("Executing planned trajectory...")
        return True

    def run(self):
        """Main planning and execution loop"""
        # Use current position as start
        q_start = self.current_joints
        
        # Generate valid goal configuration
        q_goal = self.random_config()
        while not self.is_config_valid(q_goal):
            q_goal = self.random_config()
        
        # Plot start and goal
        start_pos = self.forward_kinematics(q_start)
        goal_pos = self.forward_kinematics(q_goal)
        self.ax.scatter(start_pos[0], start_pos[1], start_pos[2], c='b', s=100, label='Start')
        self.ax.scatter(goal_pos[0], goal_pos[1], goal_pos[2], c='r', s=100, label='Goal')
        self.ax.legend()
        plt.pause(0.1)
        
        # Plan path
        path = self.rrt_plan(q_start, q_goal)
        
        if path:
            rospy.loginfo(f"Path found with {len(path)} waypoints")
            self.execute_trajectory(path)
            plt.show()
        else:
            rospy.logerr("No valid path found")

class Node:
    def __init__(self, config, parent=None):
        self.config = np.array(config)
        self.parent = parent
        
    def __eq__(self, other):
        return np.allclose(self.config, other.config)

if __name__ == '__main__':
    try:
        planner = KUKAiiwaRRTPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Planning interrupted")