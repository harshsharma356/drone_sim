import rclpy 
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys, termios, tty, select, time, math


class DroneTeleop(Node):
    def __init__(self):
        super().__init__("drone_teleop")
        self.publisher = self.create_publisher(Actuators, '/motor_speed', 10)

        self.latest_imu = None
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.latest_odom = None
        self.target_position = None
        self.max_command_speed = 0.5
        self.key_timeout = 0.25
        self.key_last_seen = {}
        self.command_velocity = [0.0, 0.0, 0.0]
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)



        self.desired_roll_rate = 0.0
        self.roll_integral = 0.0
        self.previous_roll_error = None
        self.previous_control_time = time.monotonic()
        self.motor_speed = 0.0

        self.kp_roll = 1.0
        self.ki_roll = 1.0
        self.kd_roll = 1.0

        self.teminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.timer = self.create_timer(0.1, self.publish_motor_speed)

        self.position_error = [0.0, 0.0, 0.0]
        self.kp_altitude = 1.0
        self.kd_altitude = 1.0
        self.vertical_acceleration_command = 0.0
        self.previous_odom_z = None


    def publish_motor_speed(self):

        control_time = time.monotonic()
        dt = control_time - self.previous_control_time
        self.previous_control_time = control_time


        while  select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()

            if key in ('w', 's', 'a', 'd', 'q', 'e'):
                self.key_last_seen[key] = time.monotonic()

        now = time.monotonic()
        self.key_last_seen = {
            key: stamp for key, stamp in self.key_last_seen.items()
            if now - stamp < self.key_timeout

        }

        vx = 0.0
        vy = 0.0
        vz = 0.0

        if 'w' in self.key_last_seen:
            vx += 1.0

        if 's' in self.key_last_seen:
            vx -= 1.0

        if 'a' in self.key_last_seen:
            vy += 1.0

        if 'd' in self.key_last_seen:
            vy -= 1.0

        if 'q' in self.key_last_seen:
            vz += 1.0

        if 'e' in self.key_last_seen:
            vz -= 1.0


        magnitude = (vx**2 + vy**2 + vz**2)**0.5
        if magnitude >0.0:
            scale = self.max_command_speed / magnitude
            vx *= scale
            vy *= scale
            vz *= scale


        self.command_velocity = [vx, vy, vz]

        if self.target_position is not None:
            self.target_position[0] += self.command_velocity[0] * dt
            self.target_position[1] += self.command_velocity[1] * dt
            self.target_position[2] += self.command_velocity[2] * dt


        if self.latest_odom is not None and self.target_position is not None:
            current = self.latest_odom.pose.pose.position
            self.position_error = [
                self.target_position[0] - current.x,
                self.target_position[1] - current.y,
                self.target_position[2] - current.z
            ]


        if self.latest_odom is not None and self.target_position is not None:
            current_z = self.latest_odom.pose.pose.position.z

            if self.previous_odom_z is None or dt<=0.0:
                measured_vertical_velocity = 0.0
            else:
                measured_vertical_velocity = (current_z - self.previous_odom_z)/dt

            self.previous_odom_z = current_z

            vertical_velocity_error = (self.command_velocity[2] - measured_vertical_velocity)

            p_altitude = self.kp_altitude * self.position_error[2]
            d_altitude = self.kd_altitude * vertical_velocity_error
            self.vertical_acceleration_command = p_altitude + d_altitude


        msg = Actuators()
        msg.velocity = [self.motor_speed] * 4  
        self.publisher.publish(msg)


        if self.latest_imu is None:
            return

        measured_roll_rate  = self.latest_imu.angular_velocity.x
        roll_error = self.desired_roll_rate - measured_roll_rate

        p_output = self.kp_roll * roll_error

        self.roll_integral += roll_error * dt
        i_output = self.ki_roll * self.roll_integral

        if self.previous_roll_error is None:
            error_rate = 0.0
        else:
            error_rate = (roll_error - self.previous_roll_error) / dt

        d_output = self.kd_roll * error_rate
        self.previous_roll_error = roll_error

        pid_output = p_output + i_output + d_output






    def reset_terminal_settings(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.teminal_settings)

    def imu_callback(self, msg):
        self.latest_imu = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

        if self.target_position is None:
            position = msg.pose.pose.position
            self.target_position = [position.x, position.y, position.z]

    






def main(args=None):
    rclpy.init(args=args)
    node = DroneTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_terminal_settings()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
