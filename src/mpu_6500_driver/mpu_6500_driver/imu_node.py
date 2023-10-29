import rclpy
from rclpy.node import Node
import time
import smbus
import struct
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

ADDR = None
bus = None
IMU_FRAME = None


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('i2c_address', '0x68')

        self.publisher_temp = self.create_publisher(Temperature, 'imu_temp', 10)
        self.publisher_imu = self.create_publisher(Imu, 'imu_raw', 10)

        self.setup()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def setup(self):
        bus = smbus.SMBus(1)
        ADDR = 0x68
        if type(ADDR) == str:
            ADDR = int(ADDR, 16)
        IMU_FRAME = 'imu_link'

        bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    def timer_callback(self):
        self.publish_temp()
        self.publish_imu()
        
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def read_word(self, adr):
        high = bus.read_byte_data(ADDR, adr)
        low = bus.read_byte_data(ADDR, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def publish_temp(self):
        temp_msg = Temperature()
        temp_msg.header.frame_id = IMU_FRAME
        temp_msg.temperature = self.self.read_word_2c(TEMP_H)/340.0 + 36.53
        temp_msg.header.stamp = Node.get_clock().now().to_msg()
        self.publisher_temp.publish(temp_msg)


    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.frame_id = IMU_FRAME

        # Read the acceleration vals
        accel_x = self.read_word_2c(ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(ACCEL_ZOUT_H) / 16384.0
        
        # Calculate a quaternion representing the orientation
        accel = accel_x, accel_y, accel_z
        ref = np.array([0, 0, 1])
        acceln = accel / np.linalg.norm(accel)
        axis = np.cross(acceln, ref)
        angle = np.arccos(np.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)

        # Read the gyro vals
        gyro_x = self.read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = self.read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_2c(GYRO_ZOUT_H) / 131.0

        roll = atan2(acc_y, acc_z);
        
        # Load up the IMU message
        o = imu_msg.orientation
        o.x, o.y, o.z, o.w = orientation

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.header.stamp = Node.get_clock().now().to_msg()

        self.publisher_imu.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
