#!/usr/bin/env python3
import math, time, uuid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from application_manager_interfaces.action import DeploymentRequest
from application_manager_interfaces.msg import Application, ApplicationHeader

NEAR_THR = 50.0   # deploy below this value
FAR_THR  = 100.0  # shutdown above this value
GOAL_COOLDOWN_S = 3.0  # avoid goal spam to AM

class DistanceEventDetector(Node):
    def __init__(self):
        super().__init__('distance_event_detector')

        # PX4 SITL #1 and #2
        self.sub1 = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.cb1, qos_profile_sensor_data)
        self.sub2 = self.create_subscription(
            VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry',
            self.cb2, qos_profile_sensor_data)

        self.p1 = None
        self.p2 = None
        self.is_near = False
        self.last_goal_ts = 0.0

        # AM executor action server
        self.am_action_name = '/deployment_request'
        self.am_client = ActionClient(self, DeploymentRequest, self.am_action_name)

        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz
        self.get_logger().info('Event Detector started (NEAR<50m / FAR>100m)')

    def cb1(self, msg: VehicleOdometry):
        self.p1 = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))

    def cb2(self, msg: VehicleOdometry):
        self.p2 = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))

    def tick(self):
        if self.p1 is None or self.p2 is None:
            return
        d = math.dist(self.p1, self.p2)

        # hysteresis transitions
        if (not self.is_near) and d < NEAR_THR:
            self.is_near = True
            self.get_logger().info(f'NEAR: d={d:.1f} m → DEPLOY data_recorder')
            self.send_deployment_request(start=True)
        elif self.is_near and d > FAR_THR:
            self.is_near = False
            self.get_logger().info(f'FAR : d={d:.1f} m → SHUTDOWN data_recorder')
            self.send_deployment_request(start=False)

    def send_deployment_request(self, start: bool):
        # anti-spam cooldown
        now = time.time()
        if now - self.last_goal_ts < GOAL_COOLDOWN_S:
            return
        self.last_goal_ts = now

        if not self.am_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Action server '{self.am_action_name}' not reachable")
            return

        goal = DeploymentRequest.Goal()
        goal.id = str(uuid.uuid4())
        goal.shutdown = (not start)

        app = Application()
        app.header = ApplicationHeader()
        app.header.id = 'data_recorder'          # must match AM_APP_MAP
        app.header.node_id = 'data-recorder-1'   # optional
        goal.apps = [app]
        goal.connections = []                    # not needed here

        send_future = self.am_client.send_goal_async(goal)
        def _done_cb(_):
            self.get_logger().info(f"Goal sent (shutdown={goal.shutdown})")
        send_future.add_done_callback(_done_cb)

def main():
    rclpy.init()
    node = DistanceEventDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
