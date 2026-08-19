#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.expanduser('~/av_control_guide/python_prototype'))

import rclpy
from rclpy.node import Node
from av_control_msgs.msg import VehicleCmd, VehicleState
from models.dynamic_bicycle_model import DynamicBicycleModel
from utils.trajectory_generator import generate_s_curve


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_bridge_node')

        self.model = DynamicBicycleModel(dt=0.05)
        traj = generate_s_curve()
        self.model.set_state(traj[0, 0], traj[0, 1], traj[0, 2], 0.0)

        self.cmd_sub = self.create_subscription(
            VehicleCmd, '/vehicle_cmd', self.cmd_callback, 10)

        self.state_pub = self.create_publisher(VehicleState, '/vehicle_state', 10)
        self.timer = self.create_timer(0.05, self.publish_state)

        self.get_logger().info('仿真桥接节点已启动')

    def cmd_callback(self, msg: VehicleCmd):
        self.model.update(delta=msg.delta, a=msg.accel)

    def publish_state(self):
        state = self.model.get_state()
        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x     = float(state[0])
        msg.y     = float(state[1])
        msg.theta = float(state[2])
        msg.v     = float(state[3])
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
