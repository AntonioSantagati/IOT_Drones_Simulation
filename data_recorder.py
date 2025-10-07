#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from functools import partial
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from pymongo import MongoClient

def get_uav_id_from_topic(topic: str) -> str:
    # /fmu/out/...    -> uav1
    # /px4_1/fmu/out/ -> uav2
    return 'uav2' if topic.startswith('/px4_1/') else 'uav1'

class Recorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        mongo_uri = os.environ.get('MONGO_URI')
        if not mongo_uri:
            raise RuntimeError('MONGO_URI non impostata')

        self.client = MongoClient(mongo_uri, tlsAllowInvalidCertificates=True)

        db = self.client.get_default_database()
        if db is None:
            db = self.client['robotdb']
        self.db = db

        self.col_odom = self.db['vehicle_odometry']
        self.col_status = self.db['vehicle_status']

        self.get_logger().info(f'Connesso a Mongo: {self.db.name}')

        # Odometry (QoS sensor data) with topic passed to the callback
        self.sub_odom_1 = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            partial(self.cb_odom, topic='/fmu/out/vehicle_odometry'),
            qos_profile_sensor_data)

        self.sub_odom_2 = self.create_subscription(
            VehicleOdometry, '/px4_1/fmu/out/vehicle_odometry',
            partial(self.cb_odom, topic='/px4_1/fmu/out/vehicle_odometry'),
            qos_profile_sensor_data)

        # Status (use sensor-data QoS for consistency with PX4)
        self.sub_status_1 = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            partial(self.cb_status, topic='/fmu/out/vehicle_status_v1'),
            qos_profile_sensor_data)

        self.sub_status_2 = self.create_subscription(
            VehicleStatus, '/px4_1/fmu/out/vehicle_status_v1',
            partial(self.cb_status, topic='/px4_1/fmu/out/vehicle_status_v1'),
            qos_profile_sensor_data)

    def cb_odom(self, msg: VehicleOdometry, topic: str):
        uav = get_uav_id_from_topic(topic)
        doc = {
            'uav': uav,
            't': int(msg.timestamp),  # ns since boot
            'position': {
                'x': float(msg.position[0]),
                'y': float(msg.position[1]),
                'z': float(msg.position[2]),
            },
            'q': [float(v) for v in msg.q],
            'vx': float(msg.velocity[0]),
            'vy': float(msg.velocity[1]),
            'vz': float(msg.velocity[2]),
        }
        try:
            self.col_odom.insert_one(doc)
        except Exception as e:
            self.get_logger().warn(f'Insert odom fallita: {e}')

    def cb_status(self, msg: VehicleStatus, topic: str):
        uav = get_uav_id_from_topic(topic)
        doc = {
            'uav': uav,
            't': int(msg.timestamp),
            'nav_state': int(msg.nav_state),
            'arming_state': int(msg.arming_state),
            'failsafe': bool(msg.failsafe),
        }
        try:
            self.col_status.insert_one(doc)
        except Exception as e:
            self.get_logger().warn(f'Insert status fallita: {e}')

def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
