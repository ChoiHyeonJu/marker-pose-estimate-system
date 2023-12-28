import rclpy
import math
import csv
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion    # need to install ros-foxy-tf-transformations
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
class ExpLogger(Node):
    def __init__(self):
        super().__init__('exp_logger')
        self.declare_parameter("show_plot", True)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("deactivate_topic", "/deactivate")
        self.declare_parameter("mov_joint_states_topic", "/mov_control_node/joint_states")
        self.declare_parameter("fix_joint_states_topic", "/fix_control_node/joint_states")
        self.declare_parameter("mov_marker_detect_topic", "/mov_aruco_node/marker_detect")
        self.declare_parameter("est_odom_topic", "/odom")
        self.declare_parameter("gt_odom_topic", "/odom")
        self.declare_parameter("cmd_period", 0.02)
        self.declare_parameter("linear_vel", 0.10)
        self.declare_parameter("circle_rad", 0.6)
        self.declare_parameter("line_dist", 0.3)

        self.csv_file = open('exp_log_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
                                  'marker_detect',
                                  'deactivate',
                                  'est_x',
                                  'est_y',
                                  'est_z',
                                  'est_qx',
                                  'est_qy',
                                  'est_qz',
                                  'est_qw',
                                  'gt_x',
                                  'gt_y',
                                  'gt_z',
                                  'gt_qx',
                                  'gt_qy',
                                  'gt_qz',
                                  'gt_qw',
                                  'mov_joint1',
                                  'mov_joint2',
                                  'fix_joint1',
                                  'fix_joint2'
                                ])  # CSV 파일 헤더 작성
        self.show_plot = self.get_parameter("show_plot").get_parameter_value().bool_value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        deactivate_topic = self.get_parameter("deactivate_topic").get_parameter_value().string_value
        est_odom_topic = self.get_parameter("est_odom_topic").get_parameter_value().string_value
        gt_odom_topic = self.get_parameter("gt_odom_topic").get_parameter_value().string_value
        mov_joint_states_topic = self.get_parameter("mov_joint_states_topic").get_parameter_value().string_value
        fix_joint_states_topic = self.get_parameter("fix_joint_states_topic").get_parameter_value().string_value
        mov_marker_detect_topic = self.get_parameter("mov_marker_detect_topic").get_parameter_value().string_value
        cmd_period = self.get_parameter("cmd_period").get_parameter_value().double_value
        self.linear_vel = self.get_parameter("linear_vel").get_parameter_value().double_value
        circle_rad = self.get_parameter("circle_rad").get_parameter_value().double_value
        target_dist = self.get_parameter("line_dist").get_parameter_value().double_value

        self.angular_vel = 0.0
        if abs(circle_rad) > 0:
            self.angular_vel = self.linear_vel / circle_rad
            target_dist = math.pi * circle_rad
            # self.get_logger().info("angular vel=%.2f" % (self.angular_vel))
        # stop_time = 2 * abs(target_dist / self.linear_vel)
        stop_time = abs(target_dist / self.linear_vel)
        self.stop_max_cnt = stop_time / cmd_period
        self.stop_half_max_cnt = self.stop_max_cnt / 2
        self.stop_cnt = 0
        # self.get_logger().info("Dist=%2f, Time=%.2f" % (target_dist, stop_time))

        self.deactivate_pub = self.create_publisher(Bool, deactivate_topic, 10)
        self.deactivate_msg = Bool()
        self.deactivate_msg.data = False

        self.marker_detect_msg = Bool()


        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        # self.timer = self.create_timer(cmd_period, self.publish_cmd_vel)
        self.timer = self.create_timer(cmd_period, self.record_data)
        self.moving = True

        self.est_odom_subscriber = self.create_subscription(
            Odometry,
            est_odom_topic,
            self.est_odom_callback,
            10)
        self.gt_odom_subscriber = self.create_subscription(
            Odometry,
            gt_odom_topic,
            self.gt_odom_callback,
            10)
        self.mov_joint_subscriber = self.create_subscription(
            JointState,
            mov_joint_states_topic,
            self.mov_joint_state_callback,
            10
        )
        self.fix_joint_subscriber = self.create_subscription(
            JointState,
            fix_joint_states_topic,
            self.fix_joint_state_callback,
            10
        )
        self.mov_marker_detect_subscriber = self.create_subscription(
            Bool,
            mov_marker_detect_topic,
            self.mov_marker_detect_callback,
            10
        )
        self.est_x = 0
        self.est_y = 0
        self.est_z = 0
        self.est_qx = 0
        self.est_qy = 0
        self.est_qz = 0
        self.est_qw = 0
        self.gt_x = 0
        self.gt_y = 0
        self.gt_z = 0
        self.gt_qx = 0
        self.gt_qy = 0
        self.gt_qz = 0
        self.gt_qw = 0
        self.mov_joint1 = 0
        self.mov_joint2 = 0
        self.fix_joint1 = 0
        self.fix_joint2 = 0

        self.est_poses_x = []
        self.est_poses_y = []
        self.est_poses_z = []
        self.gt_poses_x = []
        self.gt_poses_y = []
        self.gt_poses_z = []
        # self.detects_check = []

        self.fig = plt.figure(figsize=(9, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')


    # def publish_cmd_vel(self):
    #     self.stop_cnt += 1
    #     cmd_msg = Twist()
    #     if self.stop_cnt < self.stop_max_cnt:
    #         self.csv_writer.writerow([
    #                                     int(self.marker_detect_msg.data),
    #                                     int(self.deactivate_msg.data),
    #                                     round(self.est_x, 3),
    #                                     round(self.est_y, 3),
    #                                     round(self.est_z, 3),
    #                                     round(self.gt_x, 3),
    #                                     round(self.gt_y, 3),
    #                                     round(self.gt_z, 3),
    #                                     round(self.mov_joint1, 3),
    #                                     round(self.mov_joint2, 3),
    #                                     round(self.fix_joint1, 3),
    #                                     round(self.fix_joint2, 3),
    #                                 ])
    #         cmd_msg.linear.x = self.linear_vel
    #         cmd_msg.angular.z = self.angular_vel
    #         if self.show_plot:
    #             if self.marker_detect_msg.data:
    #                 self.est_poses_x.append(self.est_x)
    #                 self.est_poses_y.append(self.est_y)
    #                 self.est_poses_z.append(self.est_z)
    #                 self.gt_poses_x.append(self.gt_x)
    #                 self.gt_poses_y.append(self.gt_y)
    #                 self.gt_poses_z.append(self.gt_z)
    #             plt.clf()
    #             plt.scatter(self.est_poses_x, self.est_poses_y, color='blue', marker='*')
    #             plt.scatter(self.gt_poses_x, self.gt_poses_y, color='red', marker='o')
    #             plt.xlabel('x')
    #             plt.ylabel('y')
    #             plt.axis('equal')
    #             plt.grid(True)
    #             plt.pause(0.01)

    #             # self.fig.clf()
    #             # self.ax.scatter(self.est_poses_x, self.est_poses_y, color='blue', marker='*')
    #             # self.ax.scatter(self.gt_poses_x, self.gt_poses_y, color='red', marker='o')
    #             # # plt.xlabel('x')
    #             # # plt.ylabel('y')
    #             # self.ax.axis('equal')
    #             # self.ax.grid(True)
    #             # self.fig.pause(0.01)


    #         if self.deactivate_msg.data == False and self.stop_cnt > self.stop_half_max_cnt:
    #             # self.linear_vel *= -1.0
    #             # self.angular_vel *= -1.0
    #             self.deactivate_msg.data = True
    #             self.get_logger().warn("Deactivate! (%d / %d)" % (self.stop_cnt, self.stop_max_cnt))
    #     elif self.moving:
    #         self.csv_file.close()
    #         self.moving = False
    #         self.get_logger().warn("Complete!")

    #     self.cmd_vel_publisher.publish(cmd_msg)
    #     self.deactivate_pub.publish(self.deactivate_msg)
    #     # self.get_logger().info("linear=%.2f, angular=%.2f" % (cmd_msg.linear.x, cmd_msg.angular.z))


    def record_data(self):
        self.csv_writer.writerow([
                                    int(self.marker_detect_msg.data),
                                    int(self.deactivate_msg.data),
                                    round(self.est_x, 3),
                                    round(self.est_y, 3),
                                    round(self.est_z, 3),
                                    round(self.est_qx, 3),
                                    round(self.est_qy, 3),
                                    round(self.est_qz, 3),
                                    round(self.est_qw, 3),
                                    round(self.gt_x, 3),
                                    round(self.gt_y, 3),
                                    round(self.gt_z, 3),
                                    round(self.gt_qx, 3),
                                    round(self.gt_qy, 3),
                                    round(self.gt_qz, 3),
                                    round(self.gt_qw, 3),
                                    round(self.mov_joint1, 3),
                                    round(self.mov_joint2, 3),
                                    round(self.fix_joint1, 3),
                                    round(self.fix_joint2, 3),
                                ])
        if self.show_plot:
            if self.marker_detect_msg.data:
                self.est_poses_x.append(self.est_x)
                self.est_poses_y.append(self.est_y)
                self.est_poses_z.append(self.est_z)
                self.gt_poses_x.append(self.gt_x)
                self.gt_poses_y.append(self.gt_y)
                self.gt_poses_z.append(self.gt_z)
            plt.clf()
            plt.scatter(self.est_poses_x, self.est_poses_y, color='blue', marker='*')
            plt.scatter(self.gt_poses_x, self.gt_poses_y, color='red', marker='o')
            plt.xlabel('x')
            plt.ylabel('y')
            plt.axis('equal')
            plt.grid(True)
            plt.pause(0.01)




    def est_odom_callback(self, msg):
        pose = msg.pose.pose
        # self.get_logger().info(" GT pos: x=%.2f, y=%.2f, z=%.2f" % (pose.position.x, pose.position.y, pose.position.z))
        # self.get_logger().info("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f" % (
        #     orientation.x, orientation.y, orientation.z, orientation.w))
        self.est_x = pose.position.x
        self.est_y = pose.position.y
        self.est_z = pose.position.z
        self.est_qx = pose.orientation.x
        self.est_qy = pose.orientation.y
        self.est_qz = pose.orientation.z
        self.est_qw = pose.orientation.w

        # quaternion = (
        #     pose.orientation.x,
        #     pose.orientation.y,
        #     pose.orientation.z,
        #     pose.orientation.w
        # )
        # _, _, self.est_z = euler_from_quaternion(quaternion)
        # self.get_logger().info("x=%.2f, y=%.2f, yaw=%.2f" % (x, y, yaw))


    def gt_odom_callback(self, msg):
        pose = msg.pose.pose
        # self.get_logger().warn(" ET pos: x=%.2f, y=%.2f, z=%.2f" % (pose.position.x, pose.position.y, pose.position.z))
        # self.get_logger().info("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f" % (
        #     orientation.x, orientation.y, orientation.z, orientation.w))
        self.gt_x = pose.position.x
        self.gt_y = pose.position.y
        self.gt_z = pose.position.z
        self.gt_qx = pose.orientation.x
        self.gt_qy = pose.orientation.y
        self.gt_qz = pose.orientation.z
        self.gt_qw = pose.orientation.w
        # quaternion = (
        #     pose.orientation.x,
        #     pose.orientation.y,
        #     pose.orientation.z,
        #     pose.orientation.w
        # )
        # _, _, self.gt_z = euler_from_quaternion(quaternion)
        # self.get_logger().info("x=%.2f, y=%.2f, yaw=%.2f" % (x, y, yaw))


    def mov_joint_state_callback(self, msg):
        # joint_names = msg.name
        # joint_positions = msg.position
        # self.get_logger().info("Received mov JointState message:")
        # for name, pos in zip(joint_names, joint_positions):
        #     self.get_logger().info("Joint: %s, Position: %.2f" % (name, pos))

        self.mov_joint1 = msg.position[0]
        self.mov_joint2 = msg.position[1]
        # self.get_logger().warn("Tot Joint: %.2f, %.2f" % (self.mov_joint1, self.mov_joint2))


    def fix_joint_state_callback(self, msg):
        # joint_names = msg.name
        # joint_positions = msg.position
        # self.get_logger().info("Received fix JointState message:")
        # for name, pos in zip(joint_names, joint_positions):
        #     self.get_logger().info("Joint: %s, Position: %.2f" % (name, pos))

        self.fix_joint1 = msg.position[0]
        self.fix_joint2 = msg.position[1]
        # self.get_logger().warn("Tot Joint: %.2f, %.2f" % (self.fix_joint1, self.fix_joint2))


    def mov_marker_detect_callback(self, msg):
        self.marker_detect_msg.data = msg.data


def main(args=None):
    rclpy.init(args=args)
    exp_logger = ExpLogger()
    rclpy.spin(exp_logger)
    exp_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
