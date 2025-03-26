#!/usr/bin/python3

import rospy
import random
import numpy as np
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

class RRTPlanner:
    def __init__(self):
        rospy.init_node('rrt_motion_planner')
        
        # Publisher for joint commands
        self.pub = rospy.Publisher('/iiwa/PositionJointInterface_trajectory_controller/command',
                                 JointTrajectory,
                                 queue_size=10)
        
        # Subscriber for current joint states
        self.joint_state_sub = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback)
        
        # Joint names and limits for KUKA iiwa (7 DOF)
        self.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',
                          'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6',
                          'iiwa_joint_7']
        
        self.joint_limits = np.array([
            [-2.96, 2.96],    # Joint 1
            [-2.09, 2.09],    # Joint 2
            [-2.96, 2.96],    # Joint 3
            [-2.09, 2.09],    # Joint 4
            [-2.96, 2.96],    # Joint 5
            [-2.09, 2.09],    # Joint 6
            [-3.05, 3.05]     # Joint 7
        ])
        
        # Current joint positions
        self.current_joints = None
        self.got_initial_state = False
        
        # RRT Parameters
        self.max_iterations = 500
        self.step_size = 0.1  # Radians
        self.goal_bias = 0.1   # 10% chance to sample goal directly
        self.goal_threshold = 0.2  # Radians
        
        # Collision checking (simple joint limit checking)
        self.safety_margin = 0.1  # Radians
    
    def joint_state_callback(self, msg):
        """Get current joint positions"""
        if not self.got_initial_state:
            self.current_joints = np.zeros(len(self.joint_names))
            for i, name in enumerate(self.joint_names):
                try:
                    idx = msg.name.index(name)
                    self.current_joints[i] = msg.position[idx]
                except ValueError:
                    rospy.logwarn(f"Joint {name} not found in joint_states")
            
            if all(p is not None for p in self.current_joints):
                self.got_initial_state = True
                rospy.loginfo(f"Initial joints: {self.current_joints}")
    
    def is_state_valid(self, joints):
        """Check if joint configuration is valid (within limits and collision-free)"""
        # Check joint limits
        for i in range(len(joints)):
            if (joints[i] < self.joint_limits[i, 0] + self.safety_margin or 
                joints[i] > self.joint_limits[i, 1] - self.safety_margin):
                return False
        
        # Add more sophisticated collision checking here if needed
        # (e.g., self-collision checking using robot URDF)
        
        return True
    
    def distance(self, a, b):
        """Calculate Euclidean distance between two joint configurations"""
        return np.linalg.norm(np.array(a) - np.array(b))
    
    def sample_random_config(self):
        """Generate a random joint configuration within limits"""
        return np.array([random.uniform(low + self.safety_margin, high - self.safety_margin)
                        for low, high in self.joint_limits])
    
    def find_nearest_neighbor(self, tree, target):
        """Find nearest node in tree to target"""
        distances = [self.distance(node, target) for node in tree]
        return np.argmin(distances)
    
    def steer(self, from_node, to_node):
        """Move from 'from_node' toward 'to_node' by step_size"""
        direction = np.array(to_node) - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance < self.step_size:
            return to_node
        else:
            return from_node + (direction / distance) * self.step_size
    
    def rrt_plan(self, start, goal):
        """RRT planning algorithm"""
        tree = [start]
        parent_map = {0: -1}  # Root node has no parent
        iterations = 0
        start_time = time.time()
        
        for iterations in range(1, self.max_iterations + 1):
            # Sample random configuration (with goal biasing)
            if random.random() < self.goal_bias:
                rand_config = goal
            else:
                rand_config = self.sample_random_config()
            
            # Find nearest node in tree
            nearest_idx = self.find_nearest_neighbor(tree, rand_config)
            nearest_node = tree[nearest_idx]
            
            # Steer toward random configuration
            new_node = self.steer(nearest_node, rand_config)
            
            # Check if path is valid
            if self.is_state_valid(new_node):
                # Add to tree
                tree.append(new_node)
                parent_map[len(tree)-1] = nearest_idx
                
                # Check if reached goal
                if self.distance(new_node, goal) < self.goal_threshold:
                    end_time = time.time()
                    rospy.loginfo(f"RRT planning successful in {iterations} iterations")
                    rospy.loginfo(f"Planning time: {end_time - start_time:.2f} seconds")
                    return self.reconstruct_path(tree, parent_map)
        
        end_time = time.time()
        rospy.logwarn(f"RRT failed after {iterations} iterations")
        rospy.logwarn(f"Planning time: {end_time - start_time:.2f} seconds")
        return None
    
    def reconstruct_path(self, tree, parent_map):
        """Reconstruct path from tree"""
        path = []
        current_idx = len(tree) - 1
        
        while current_idx != -1:
            path.insert(0, tree[current_idx])
            current_idx = parent_map[current_idx]
        
        return path
    
    def execute_trajectory(self, path):
        """Execute planned path"""
        if not path:
            rospy.logerr("Empty path - cannot execute")
            return
        
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.joint_names
        
        duration = 0.0
        time_step = 1.0  # Time between points (seconds)
        
        for i, point in enumerate(path):
            traj_point = JointTrajectoryPoint()
            traj_point.positions = point
            traj_point.time_from_start = rospy.Duration(duration)
            traj.points.append(traj_point)
            duration += time_step
        
        self.pub.publish(traj)
        rospy.loginfo("Executing planned trajectory...")
    
    def run(self):
        rospy.loginfo("Waiting for initial joint state...")
        while not rospy.is_shutdown() and not self.got_initial_state:
            rospy.sleep(0.1)
        
        if self.current_joints is None:
            rospy.logerr("Failed to get initial joint state!")
            return
        
        # Generate random start and goal (or use current position as start)
        start = self.current_joints
        goal = self.sample_random_config()
        
        rospy.loginfo(f"Planning from: {start}")
        rospy.loginfo(f"Planning to: {goal}")
        
        # Run RRT planner
        path = self.rrt_plan(start, goal)
        
        if path:
            self.execute_trajectory(path)
            rospy.loginfo("Motion execution complete")
        else:
            rospy.logerr("No valid path found")

if __name__ == '__main__':
    try:
        planner = RRTPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Planning interrupted")
