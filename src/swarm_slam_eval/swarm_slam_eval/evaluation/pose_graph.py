#!/usr/bin/env python3
import rclpy
import numpy as np
import os
import threading
import copy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from ..qos_profiles import DATA_QOS

class PoseGraphNode(Node):
    def __init__(self):
        super().__init__('pose_graph_node')

        namespace = self.get_namespace()
        self.robot_id = int(''.join(filter(str.isdigit, namespace))) if namespace != '/' else 0

        # --- Parameters for generalization and output path ---
        self.declare_parameter('results_base_path', '')
        self.declare_parameter('graph_name', '')
        self.declare_parameter('update_time', 1.0)
        
        self.results_base_path = self.get_parameter('results_base_path').get_parameter_value().string_value
        self.graph_name = self.get_parameter('graph_name').get_parameter_value().string_value
        self.t_interval = self.get_parameter('update_time').get_parameter_value().double_value

        # Determine the topic to subscribe to based on the graph_name
        if self.graph_name == "ground_truth":
            self.odom_topic = 'gt_vis_pose'
        else:
            self.odom_topic = f'{self.graph_name}_vis_pose'

        # --- Keyframe parameters ---
        self.declare_parameter('keyframe_dist_thresh', 1.0)
        self.declare_parameter('keyframe_angle_thresh', 10.0) 
        self.dist_thresh = self.get_parameter('keyframe_dist_thresh').get_parameter_value().double_value
        self.angle_thresh_rad = np.deg2rad(self.get_parameter('keyframe_angle_thresh').get_parameter_value().double_value)

        # --- Graph data structures ---
        self.vertices = []
        self.edges = []
        self.node_id_counter = 0
        self.lock = threading.Lock()
        self.last_keyframe_pose = None
        self.latest_pose_timestamp = None
        self.start_sim_time = None 

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.odom_topic,
            self.poseCallback,
            DATA_QOS
        )

        if self.t_interval > 0:
            self.save_timer = self.create_timer(self.t_interval, self.saveCallback)
            self.get_logger().info(f"Checkpointing enabled every {self.t_interval} seconds.")
        else:
            self.get_logger().info("Periodic checkpointing disabled (save_interval_sec <= 0).")
        
        self.robot_dir = os.path.join(self.results_base_path, f"r{self.robot_id}", self.graph_name)
        os.makedirs(self.robot_dir, exist_ok=True)

        rclpy.get_default_context().on_shutdown(self.saveOnShutdown)
          
        self.get_logger().info(f"PoseGraphNode for robot {self.robot_id} tracking '{self.graph_name}'. Output: {self.results_base_path}")

    # Callback function that receives the pose from topic subscription
    def poseCallback(self, msg: PoseWithCovarianceStamped): 
        current_pose = msg.pose.pose 
        current_timestamp = msg.header.stamp

        with self.lock:
            self.latest_pose_timestamp = msg.header.stamp
            if self.start_sim_time is None:
                self.start_sim_time = msg.header.stamp
                self.get_logger().info(f"Simulation start time recorded: {self.start_sim_time.sec}s")

            if self.last_keyframe_pose is None:
                self.addVertex(current_pose, current_timestamp)
                self.last_keyframe_pose = current_pose
                return

            dist, angle = self.calculatePoseChange(self.last_keyframe_pose, current_pose)

            if dist > self.dist_thresh or angle > self.angle_thresh_rad:
                self.addVertex(current_pose, current_timestamp)
                self.addEdge(self.last_keyframe_pose, current_pose)
                self.last_keyframe_pose = current_pose

    # Callback function that save the graph in the current state
    def saveCallback(self):
        self.get_logger().info("Saving checkpoint...")
        self.saveCurrentGraph()

    # Callback function that saves the graph state on shutdown
    def saveOnShutdown(self):
        self.get_logger().info("Shutdown signal received, saving final pose graph...")
        self.saveCurrentGraph(True)

    # Save the current state of the graph 
    def saveCurrentGraph(self, final=False):
        with self.lock:
            if not self.vertices:
                self.get_logger().warn("No vertices to save. Graph is empty.")
                return
            
            vertices_to_save = copy.deepcopy(self.vertices)
            edges_to_save = copy.deepcopy(self.edges)
            
            current_time = Time.from_msg(self.latest_pose_timestamp)
            start_time = Time.from_msg(self.start_sim_time)
            elapsed_duration = current_time - start_time
            elapsed_seconds = elapsed_duration.nanoseconds / 1e9
            if not final:
                filename = f"{elapsed_seconds:.3f}" 
            else:
                filename = "final" 
            
        g2o_path = os.path.join(self.robot_dir, f"{filename}.g2o")
        tum_path = os.path.join(self.robot_dir, f"{filename}.tum")
     
        self.writeG2O(g2o_path, vertices_to_save, edges_to_save)
        self.writeTUM(tum_path, vertices_to_save)

    # Save the provided graph state to file as g2o
    def writeG2O(self, output_path, vertices_to_save, edges_to_save):
        self.get_logger().info(f"Writing {len(vertices_to_save)} vertices to {os.path.basename(output_path)}")
        info_matrix_str = "1000 0 0 0 0 0 1000 0 0 0 0 1000 0 0 0 1000 0 0 1000 0 1000"
        temp_path = output_path + ".tmp"
        
        try:
            with open(temp_path, 'w') as f:
                for v in vertices_to_save:
                    pose_str = " ".join(map(str, v['pose']))
                    f.write(f"VERTEX_SE3:QUAT {v['id']} {pose_str}\n")
                
                for e in edges_to_save:
                    transform_str = " ".join(map(str, e['transform']))
                    f.write(f"EDGE_SE3:QUAT {e['id_from']} {e['id_to']} {transform_str} {info_matrix_str}\n")
            
            os.rename(temp_path, output_path)
        except IOError as e:
            self.get_logger().error(f"Failed to write to file {output_path}: {e}")

    # Save the provided graph state to file as tum
    def writeTUM(self, output_path, vertices):
        self.get_logger().info(f"Saving TUM trajectory with {len(vertices)} poses to {os.path.basename(output_path)}")
        start_time = Time.from_msg(self.start_sim_time)
        with open(output_path, 'w') as f:
            for v in vertices:
                current_time = Time.from_msg(v['stamp'])
                elapsed_duration = current_time - start_time
                elapsed_seconds = elapsed_duration.nanoseconds / 1e9
                
                pose = v['pose'] # [tx, ty, tz, qx, qy, qz, qw]
                # Format: timestamp tx ty tz qx qy qz qw
                f.write(f"{elapsed_seconds:.6f} {' '.join(map(str, pose))}\n")

    # Create vertex based on pose and time
    def addVertex(self, pose, stamp):
        p = pose.position
        q = pose.orientation
        vertex = {
            'id': self.node_id_counter,
            'stamp': stamp,
            'pose': [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        }
        self.vertices.append(vertex)
        self.node_id_counter += 1

    # Create edge and add to list from two poses
    def addEdge(self, pose_from, pose_to):
        T_world_from = self.poseToMatrix(pose_from)
        T_world_to = self.poseToMatrix(pose_to)
        T_from_to = np.linalg.inv(T_world_from) @ T_world_to
        rel_pos = T_from_to[0:3, 3]
        rel_rot = Rotation.from_matrix(T_from_to[0:3, 0:3]).as_quat()
        edge = {
            'id_from': self.node_id_counter - 2,
            'id_to': self.node_id_counter - 1,
            'transform': [rel_pos[0], rel_pos[1], rel_pos[2], rel_rot[0], rel_rot[1], rel_rot[2], rel_rot[3]]
        }
        self.edges.append(edge)

    # Transform pose to matrix
    def poseToMatrix(self, pose):
        p = pose.position; q = pose.orientation
        T = np.eye(4)
        T[0:3, 0:3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[0:3, 3] = [p.x, p.y, p.z]; 
        return T

    # Based on two provided poses return the distance and angle 
    def calculatePoseChange(self, pose1, pose2):
        p1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
        p2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
        dist = np.linalg.norm(p1 - p2)
        q1 = pose1.orientation; q2 = pose2.orientation
        r1 = Rotation.from_quat([q1.x, q1.y, q1.z, q1.w])
        r2 = Rotation.from_quat([q2.x, q2.y, q2.z, q2.w])
        delta_rotation = r1.inv() * r2
        angle = delta_rotation.magnitude()
        return dist, angle

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()