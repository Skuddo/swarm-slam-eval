#!/usr/bin/env python3
import os
import rclpy
from typing import List, Tuple, Union
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time as MsgTime
from cslam_common_interfaces.msg import PoseGraph, RobotIds
from swarm_slam_eval.qos_profiles import DATA_QOS

class PoseGraphNode(Node):

    def __init__(self):
        super().__init__('pose_graph_node')

        self.declare_parameter('results_base_path', '')
        self.declare_parameter('graph_name', '')
        self.declare_parameter('update_time', 5.0)
        self.declare_parameter('tum_timestamp_mode', 'absolute')

        self.results_base_path = self.get_parameter('results_base_path').get_parameter_value().string_value
        self.graph_name = self.get_parameter('graph_name').get_parameter_value().string_value
        self.update_interval = self.get_parameter('update_time').get_parameter_value().double_value
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.tum_timestamp_mode = self.get_parameter('tum_timestamp_mode').get_parameter_value().string_value.lower()
        if self.tum_timestamp_mode not in ('elapsed', 'absolute', 'save_time'):
            self.get_logger().warn(f"Invalid tum_timestamp_mode '{self.tum_timestamp_mode}', defaulting to 'absolute'")
            self.tum_timestamp_mode = 'absolute'

        self.vertices: List[dict] = []
        self.edges: List[dict] = []
        self.gt_pose_counter = 0
        self.last_saved_tick = -1

        if self.graph_name != 'cslam_global':
            self.rid = int(self.get_namespace()[2])
            robot_subdir = f"r{self.rid}"
        else:
            robot_subdir = ""

        self.start_time: Union[MsgTime, None] = None

        self.output_dir = os.path.join(self.results_base_path, robot_subdir, self.graph_name)
        os.makedirs(self.output_dir, exist_ok=True)

        if self.graph_name == 'ground_truth':
            self.create_subscription(PoseWithCovarianceStamped, 'gt_vis_pose', self.gt_callback, DATA_QOS)
            self.get_logger().info(f"Saver started in 'ground_truth' mode for r{self.rid} (output: {self.output_dir})")
        elif self.graph_name == 'cslam':
            self.trigger_pub = self.create_publisher(RobotIds, 'cslam/get_pose_graph', DATA_QOS)
            self.create_subscription(PoseGraph, '/cslam/pose_graph', self.local_cslam_graph_callback, DATA_QOS)
            self.create_timer(self.update_interval, self.request_local_cslam_graph)
            self.get_logger().info(f"Saver started in 'cslam' (local) mode for r{self.rid}.")
        elif self.graph_name == 'cslam_global':
            self.create_subscription(PoseGraph, '/cslam/pose_graph', self.global_cslam_graph_callback, DATA_QOS)
            self.get_logger().info("Saver started in 'cslam_global' mode.")

        rclpy.get_default_context().on_shutdown(self.save_on_shutdown)

    def gt_callback(self, msg: PoseWithCovarianceStamped):
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            stamp_msg = msg.header.stamp

            if self.start_time is None:
                self.start_time = stamp_msg
                self.get_logger().info(f"start_time initialized from GT stamp: {self.start_time.sec}.{self.start_time.nanosec}")

            self.vertices.append({
                'id': self.gt_pose_counter,
                'stamp': stamp_msg,
                'pose': [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            })
            self.gt_pose_counter += 1

            self.check_and_save_at_interval(stamp_msg)

        except Exception as e:
            self.get_logger().error(f"Exception in gt_callback: {e}")

    def global_cslam_graph_callback(self, msg: PoseGraph):
        reception_time = self.get_clock().now().to_msg()
        if self.start_time is None:
            self.start_time = reception_time
            self.get_logger().debug(f"start_time initialized from global graph reception: {self.start_time.sec}.{self.start_time.nanosec}")

        self.get_logger().debug(f"Received global graph update with {len(msg.values)} vertices.")
        self.vertices, self.edges = self.parse_graph_msg(msg, reception_time, is_global=True)
        if self.start_time is None and self.vertices:
            try:
                min_stamp = min(self.vertices, key=lambda vv: (getattr(vv['stamp'], 'sec', 0), getattr(vv['stamp'], 'nanosec', 0)))['stamp']
                self.start_time = min_stamp
                self.get_logger().debug(f"start_time set from earliest vertex stamp: {self.start_time.sec}.{self.start_time.nanosec}")
            except Exception:
                self.start_time = reception_time
                self.get_logger().debug("start_time fallback to reception_time in global_cslam_graph_callback")

        self.check_and_save_at_interval(reception_time)

    def local_cslam_graph_callback(self, msg: PoseGraph):
        try:
            if hasattr(msg, 'rid') and isinstance(msg.rid, int) and msg.rid != self.rid:
                self.get_logger().debug(f"Ignoring PoseGraph for rid={msg.rid} (this saver is r{self.rid}).")
                return
        except Exception as e:
            self.get_logger().debug(f"Error while checking msg.rid: {e}")

        reception_time = self.get_clock().now().to_msg()
        if self.start_time is None:
            self.start_time = reception_time
            self.get_logger().info(f"start_time initialized from local graph reception: {self.start_time.sec}.{self.start_time.nanosec}")

        try:
            n_vals = len(msg.values) if hasattr(msg, 'values') else 0
            n_edges = len(msg.edges) if hasattr(msg, 'edges') else 0
        except Exception:
            n_vals, n_edges = 0, 0
        self.get_logger().debug(f"r{self.rid} received PoseGraph (values={n_vals}, edges={n_edges}) at {reception_time.sec}.{reception_time.nanosec}")

        local_vertices, local_edges = self.parse_graph_msg(msg, reception_time, is_local_rid=self.rid)

        if self.start_time is None and local_vertices:
            try:
                min_stamp = min(local_vertices, key=lambda vv: (getattr(vv['stamp'], 'sec', 0), getattr(vv['stamp'], 'nanosec', 0)))['stamp']
                self.start_time = min_stamp
                self.get_logger().info(f"start_time initialized from local earliest vertex stamp: {self.start_time.sec}.{self.start_time.nanosec}")
            except Exception:
                self.start_time = reception_time
                self.get_logger().info("start_time fallback to reception_time in local_cslam_graph_callback")

        if not local_vertices:
            self.get_logger().debug(f"No local vertices parsed for r{self.rid} from incoming PoseGraph (len(values)={n_vals}).")
            return

        self.vertices = local_vertices
        self.edges = local_edges

        if self.last_saved_tick == -1:
            try:
                forced_name = f"{0:.3f}"
                self.get_logger().info(f"r{self.rid}: first local graph received — forcing initial save to {forced_name}.tum")
                self.write_files(forced_name, self.vertices, self.edges)
                self.last_saved_tick = 0
            except Exception as e:
                self.get_logger().error(f"r{self.rid}: forced initial save failed: {e}")

        self.check_and_save_at_interval(reception_time)

    def request_local_cslam_graph(self):
        self.get_logger().debug(f"Requesting graph update for r{self.rid}...")
        msg = RobotIds()
        msg.ids.append(self.rid)
        try:
            self.trigger_pub.publish(msg)
            self.get_logger().debug(f"Published cslam/get_pose_graph request for r{self.rid}.")
        except Exception as e:
            self.get_logger().warn(f"Failed to publish cslam/get_pose_graph for r{self.rid}: {e}")

    def check_and_save_at_interval(self, current_timestamp: Union[MsgTime, float]):
        if self.update_interval <= 0 or not self.vertices:
            return

        try:
            if isinstance(current_timestamp, (int, float)):
                sim_time_sec = float(current_timestamp)
            else:
                sim_time_sec = float(current_timestamp.sec) + float(current_timestamp.nanosec) / 1e9
        except Exception as e:
            self.get_logger().warn(f"Failed to parse timestamp in check_and_save_at_interval: {e}")
            sim_time_sec = 0.0

        current_tick = int(sim_time_sec // self.update_interval)
        if current_tick > self.last_saved_tick:
            self.last_saved_tick = current_tick
            filename = f"{current_tick * self.update_interval:.3f}"
            self.write_files(filename, self.vertices, self.edges)

    def save_on_shutdown(self):
        self.get_logger().info(f"Final save for '{self.graph_name}'.")
        self.write_files("final", self.vertices, self.edges)

    def parse_graph_msg(self, graph_msg: PoseGraph, timestamp: MsgTime, is_global=False, is_local_rid=None) -> Tuple[List[dict], List[dict]]:
        vertices: List[dict] = []
        edges: List[dict] = []
        key_to_id_map = {(v.key.robot_id, v.key.keyframe_id): i for i, v in enumerate(graph_msg.values)}

        found_value_stamp = False

        for i, v in enumerate(graph_msg.values):
            if is_local_rid is not None and v.key.robot_id != is_local_rid:
                continue

            p, q = v.pose.position, v.pose.orientation

            verts_id = i if is_global else v.key.keyframe_id

            vertex_stamp = None
            if hasattr(v, 'stamp') and v.stamp is not None:
                vertex_stamp = v.stamp
            elif hasattr(v, 'header') and hasattr(v.header, 'stamp'):
                vertex_stamp = v.header.stamp
            elif hasattr(v, 'timestamp') and v.timestamp:
                vertex_stamp = v.timestamp
            if vertex_stamp is None:
                vertex_stamp = timestamp
            else:
                found_value_stamp = True

            vertices.append({'id': verts_id, 'stamp': vertex_stamp, 'pose': [p.x, p.y, p.z, q.x, q.y, q.z, q.w]})

        for e in graph_msg.edges:
            p, q = e.measurement.position, e.measurement.orientation
            from_key = (e.key_from.robot_id, e.key_from.keyframe_id)
            to_key = (e.key_to.robot_id, e.key_to.keyframe_id)
            if is_global:
                if from_key in key_to_id_map and to_key in key_to_id_map:
                    edges.append({'id_from': key_to_id_map[from_key], 'id_to': key_to_id_map[to_key], 'transform': [p.x, p.y, p.z, q.x, q.y, q.z, q.w]})
            elif is_local_rid is not None:
                if e.key_from.robot_id == is_local_rid and e.key_to.robot_id == is_local_rid:
                    edges.append({'id_from': e.key_from.keyframe_id, 'id_to': e.key_to.keyframe_id, 'transform': [p.x, p.y, p.z, q.x, q.y, q.z, q.w]})

        if not found_value_stamp:
            self.get_logger().debug("parse_graph_msg: no per-value stamps found; using reception timestamp for vertex stamps")

        return vertices, edges


    def save_graph_to_file(self, is_final: bool):
        if not self.vertices:
            self.get_logger().warn(f"No vertices to save for '{self.graph_name}'.")
            return

        last_stamp = self.vertices[-1]['stamp']
        try:
            sim_time_sec = float(last_stamp.sec) + float(last_stamp.nanosec) / 1e9
        except Exception:
            sim_time_sec = 0.0

        filename = "final" if is_final else f"{sim_time_sec:.3f}"
        self.write_files(filename, self.vertices, self.edges)

    def write_files(self, name: str, vertices: list, edges: list):
        tum_path = os.path.join(self.output_dir, f"{name}.tum")
        g2o_path = os.path.join(self.output_dir, f"{name}.g2o")
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(f"Saving {len(vertices)} vertices and {len(edges)} edges to {self.output_dir}/{name}.g2o")

        with open(tum_path, 'w') as f_tum:
            save_time_msg = self.get_clock().now().to_msg()
            save_time_s = float(save_time_msg.sec) + float(save_time_msg.nanosec) / 1e9

            verts_secs = []
            for v in vertices:
                try:
                    t_v = RclpyTime.from_msg(v['stamp'])
                    verts_secs.append(t_v.nanoseconds / 1e9)
                except Exception:
                    pass

            min_v = min(verts_secs) if verts_secs else None
            max_v = max(verts_secs) if verts_secs else None
            self.get_logger().info(f"Saving {len(vertices)} vertices to {tum_path} at save_time={save_time_s:.6f}, min_vertex_ts={min_v}, max_vertex_ts={max_v}")

            for v in vertices:
                try:
                    t_v = RclpyTime.from_msg(v['stamp'])
                    t_v_s = t_v.nanoseconds / 1e9
                except Exception as e:
                    t_v_s = 0.0

                if self.tum_timestamp_mode == 'elapsed':
                    if self.start_time is None:
                        elapsed_s = 0.0
                    else:
                        try:
                            t_start = RclpyTime.from_msg(self.start_time)
                            elapsed_s = (t_v - t_start).nanoseconds / 1e9
                        except Exception:
                            elapsed_s = max(0.0, t_v_s - (min_v if min_v is not None else 0.0))
                    ts_out = elapsed_s
                elif self.tum_timestamp_mode == 'save_time':
                    ts_out = save_time_s
                else:
                    ts_out = t_v_s

                if ts_out < 0:
                    ts_out = 0.0
                pose_vals = ' '.join(map(str, v['pose']))
                f_tum.write(f"{ts_out:.6f} {pose_vals}\n")


        # with open(g2o_path, 'w') as f_g2o:
        #     for v in vertices:
        #         f_g2o.write(f"VERTEX_SE3:QUAT {v['id']} {' '.join(map(str, v['pose']))}\n")
        #     info = "1000 0 0 0 0 0 1000 0 0 0 0 1000 0 0 0 1000 0 0 1000 0 1000"
        #     for e in edges:
        #         f_g2o.write(f"EDGE_SE3:QUAT {e['id_from']} {e['id_to']} {' '.join(map(str, e['transform']))} {info}\n")


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
