#!/usr/bin/env python3
import os
import copy
import threading
from typing import List, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from cslam_common_interfaces.msg import PoseGraph
from cslam_common_interfaces.msg import OptimizationResult  # if available in your env
from swarm_slam_eval.qos_profiles import DATA_QOS, CSLAM_QOS
from rclpy.exceptions import ParameterAlreadyDeclaredException

class PoseGraphNode(Node):
    def __init__(self):
        super().__init__('pose_graph_node')

        # Determine namespace-based robot id
        namespace = self.get_namespace()
        if namespace and namespace not in ('', '/'):
            # e.g. '/r0' -> 0
            ns_digits = ''.join(filter(str.isdigit, namespace.lstrip('/')))
            self.robot_id = int(ns_digits) if ns_digits else -1
        else:
            self.robot_id = -1  # global node

        # Parameters
        self.declare_parameter('results_base_path', '')
        self.declare_parameter('graph_name', '')
        self.declare_parameter('update_time', 5.0)

        self.results_base_path = self.get_parameter('results_base_path').get_parameter_value().string_value
        self.graph_name = self.get_parameter('graph_name').get_parameter_value().string_value
        self.t_interval = self.get_parameter('update_time').get_parameter_value().double_value

        # Internal state
        self.lock = threading.Lock()
        self.vertices: List[dict] = []  # {'id':..., 'stamp':TimeMsg, 'pose':[x,y,z,qx,qy,qz,qw], 'robot_id':int}
        self.edges: List[dict] = []     # {'id_from':..., 'id_to':..., 'transform':[x,y,z,qx,qy,qz,qw]}
        self.node_id_counter = 0
        self.latest_pose_timestamp = None
        self.start_sim_time = None

        # Prepare storage path
        # robot_dir will include rX subfolder for per-robot nodes; global node has robot_id == -1
        robot_subdir = f"r{self.robot_id}" if self.robot_id != -1 else ""
        self.robot_dir = os.path.join(self.results_base_path, robot_subdir, self.graph_name)
        os.makedirs(self.robot_dir, exist_ok=True)

        # Timer for periodic save (only if t_interval > 0)
        if self.t_interval > 0:
            # create save callback function BEFORE we register timer or subscriptions (avoid AttributeError)
            self.save_timer = self.create_timer(self.t_interval, self.save_callback)

        # Register subscriptions depending on graph type
        # Per-robot C-SLAM optimizer estimates (OptimizationResult)
        if self.graph_name == 'cslam':
            # Use provided CSLAM_QOS if available; fallback to transient_local QoS for late-join
            try:
                qos = CSLAM_QOS
            except Exception:
                qos = QoSProfile(depth=10)
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                qos.history = HistoryPolicy.KEEP_LAST

            # subscribe to namespaced topic (this node is already in the namespace)
            self.create_subscription(OptimizationResult, 'cslam/optimized_estimates', self.cslam_optimization_callback, qos)
            self.get_logger().info(f"PER-ROBOT CSLAM SAVER for r{self.robot_id} started. Subscribing to 'cslam/optimized_estimates'.")

        # Global C-SLAM pose graph publisher (PoseGraph)
        elif self.graph_name == 'cslam_global':
            try:
                qos = CSLAM_QOS
            except Exception:
                qos = QoSProfile(depth=10)
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
                qos.history = HistoryPolicy.KEEP_LAST

            # Global topic is at root '/cslam/pose_graph'
            self.create_subscription(PoseGraph, '/cslam/pose_graph', self.cslam_global_graph_callback, qos)
            self.get_logger().info("GLOBAL CSLAM SAVER started. Subscribing to '/cslam/pose_graph'.")

        # Ground truth saver — subscribe to visualization pose topic
        elif self.graph_name == 'ground_truth':
            self.create_subscription(PoseWithCovarianceStamped, 'gt_vis_pose', self.odometry_callback, DATA_QOS)
            self.get_logger().info(f"GROUND TRUTH SAVER for r{self.robot_id} started. Subscribing to 'gt_vis_pose'.")

        # Ensure we save on shutdown
        rclpy.get_default_context().on_shutdown(self.save_on_shutdown)

    # ---------------------------
    # Callbacks
    # ---------------------------
    def cslam_optimization_callback(self, msg: OptimizationResult):
        """Handle OptimizationResult: msg.estimates is a list of per-keyframe estimates."""
        with self.lock:
            # stamp: OptimizationResult may not have header -> use current time as best-effort
            current_stamp = self.get_clock().now().to_msg()
            if self.start_sim_time is None:
                self.start_sim_time = current_stamp
            self.latest_pose_timestamp = current_stamp

            # debug summary
            try:
                robot_ids = [int(n.key.robot_id) for n in msg.estimates]
            except Exception:
                robot_ids = None
            self.get_logger().debug(f"[cslam_opt_cb] Received {len(msg.estimates)} estimates; robot_ids={robot_ids}; self.robot_id={self.robot_id}")

            # build vertex list only for this robot (use key.robot_id and key.keyframe_id)
            new_vertices = []
            for node in msg.estimates:
                try:
                    key_robot = int(node.key.robot_id)
                    kfid = int(node.key.keyframe_id)
                    p = node.pose.position
                    q = node.pose.orientation
                except Exception:
                    # skip malformed entries
                    continue

                if key_robot == self.robot_id:
                    new_vertices.append({
                        'id': kfid,
                        'stamp': current_stamp,
                        'pose': [p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                        'robot_id': key_robot
                    })

            # Merge policy: if there are new vertices for this robot, replace; otherwise keep previous
            if new_vertices:
                self.vertices = new_vertices
                self.edges = []  # optimizer currently doesn't publish edges here (use global graph for edges)
            else:
                self.get_logger().debug("[cslam_opt_cb] No estimates for this robot in received OptimizationResult (keeping previous vertices).")

    def cslam_global_graph_callback(self, msg: PoseGraph):
        """Handle global PoseGraph messages (cslam_common_interfaces/PoseGraph)
           Uses msg.values[] for vertex poses and msg.edges[] for constraints."""
        with self.lock:
            # pick header stamp if present, otherwise current time
            if hasattr(msg, 'header') and msg.header is not None:
                stamp_to_use = msg.header.stamp
            else:
                stamp_to_use = self.get_clock().now().to_msg()

            if self.start_sim_time is None:
                self.start_sim_time = stamp_to_use
            self.latest_pose_timestamp = stamp_to_use

            # Defensive extraction of vertex list (values[])
            values = getattr(msg, 'values', None)
            if values is None:
                # log what we found
                self.get_logger().warn(f"[cslam_global] Received PoseGraph without 'values' field. msg members: {dir(msg)}")
                return

            # Build a mapping from (robot_id, keyframe_id) -> g2o id
            key_map: Dict[Tuple[int,int], int] = {}
            new_vertices: List[dict] = []
            g2o_id = 0

            for v in values:
                try:
                    key_robot = int(v.key.robot_id)
                    key_kf = int(v.key.keyframe_id)
                    p = v.pose.position
                    q = v.pose.orientation
                except Exception:
                    # skip malformed
                    self.get_logger().warn("[cslam_global] Skipping malformed value entry.")
                    continue

                key = (key_robot, key_kf)
                if key not in key_map:
                    key_map[key] = g2o_id
                    new_vertices.append({
                        'id': g2o_id,
                        'stamp': stamp_to_use,
                        'pose': [p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                        'robot_id': key_robot
                    })
                    g2o_id += 1

            # Now build edges
            new_edges: List[dict] = []
            edges_field = getattr(msg, 'edges', []) or []
            for e in edges_field:
                try:
                    from_robot = int(e.key_from.robot_id)
                    from_kf = int(e.key_from.keyframe_id)
                    to_robot = int(e.key_to.robot_id)
                    to_kf = int(e.key_to.keyframe_id)
                    p = e.measurement.position
                    q = e.measurement.orientation
                except Exception:
                    self.get_logger().warn("[cslam_global] Skipping malformed edge entry.")
                    continue

                kf_from = (from_robot, from_kf)
                kf_to = (to_robot, to_kf)
                if kf_from in key_map and kf_to in key_map:
                    new_edges.append({
                        'id_from': key_map[kf_from],
                        'id_to'  : key_map[kf_to],
                        'transform': [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                    })

            # Save into state
            self.vertices, self.edges = new_vertices, new_edges

    def odometry_callback(self, msg: PoseWithCovarianceStamped):
        """Simplified ground truth saver: add each pose as a vertex."""
        with self.lock:
            if self.start_sim_time is None:
                self.start_sim_time = msg.header.stamp
            self.latest_pose_timestamp = msg.header.stamp

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.vertices.append({'id': self.node_id_counter,
                                  'stamp': msg.header.stamp,
                                  'pose': [p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                                  'robot_id': self.robot_id})
            self.node_id_counter += 1

    # ---------------------------
    # Saving
    # ---------------------------
    def save_callback(self):
        self.save_current_graph(final=False)

    def save_on_shutdown(self):
        self.save_current_graph(final=True)

    def save_current_graph(self, final=False):
        with self.lock:
            if not self.vertices or self.latest_pose_timestamp is None or self.start_sim_time is None:
                self.get_logger().warn(f"[{self.graph_name}] Save called but no data or missing stamps. Skipping.")
                return

            # compute filename: elapsed time (sec) since start_sim_time to latest_pose_timestamp
            try:
                elapsed = (Time.from_msg(self.latest_pose_timestamp) - Time.from_msg(self.start_sim_time)).nanoseconds / 1e9
                filename = "final" if final else f"{elapsed:.3f}"
            except Exception:
                filename = "final" if final else "unknown_time"

            self.write_files(self.robot_dir, filename, copy.deepcopy(self.vertices), copy.deepcopy(self.edges))

    def write_files(self, directory: str, name: str, vertices: List[dict], edges: List[dict]):
        """Write .tum and .g2o files. vertices contains 'stamp' which is a builtin_interfaces/Time msg."""
        self.get_logger().info(f"[{self.graph_name}] Saving {len(vertices)} poses to {name}.tum/g2o at {directory}")
        # .tum uses elapsed seconds relative to start_sim_time
        start_time = Time.from_msg(self.start_sim_time)
        tum_path = os.path.join(directory, f"{name}.tum")
        g2o_path = os.path.join(directory, f"{name}.g2o")

        with open(tum_path, 'w') as f_tum:
            for v in vertices:
                try:
                    elapsed_s = (Time.from_msg(v['stamp']) - start_time).nanoseconds / 1e9
                except Exception:
                    elapsed_s = 0.0
                pose_vals = ' '.join(map(str, v['pose']))
                f_tum.write(f"{elapsed_s:.6f} {pose_vals}\n")

        # g2o: VERTEX_SE3:QUAT <id> x y z qx qy qz qw
        with open(g2o_path, 'w') as f_g2o:
            for v in vertices:
                f_g2o.write(f"VERTEX_SE3:QUAT {v['id']} {' '.join(map(str, v['pose']))}\n")
            # edges: write a reasonable information matrix if not provided
            info = "1000 0 0 0 0 0 1000 0 0 0 0 1000 0 0 0 1000 0 0 1000 0 1000"
            for e in edges:
                f_g2o.write(f"EDGE_SE3:QUAT {e['id_from']} {e['id_to']} {' '.join(map(str, e['transform']))} {info}\n")

# Entrypoint
def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
