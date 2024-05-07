import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



class TurtlesimController(Node):

        def __init__(self):
            super().__init__('turtlesim_controller')
            self.twist_pub = self.create_publisher(
                            Twist, '/turtle1/cmd_vel', 10)
            self.pose = None
            self.subscription = self.create_subscription(
               Pose,
               '/turtle1/pose',
                self.update_pose,
                10)

    # New method for TurtlesimController
        def update_pose(self, msg):
             self.pose = msg
             self.pose.x = round(self.pose.x, 4)
             self.pose.y = round(self.pose.y, 4)

        def euclidean_distance(self, goal_x, goal_y):
                """Euclidean distance between current pose and the goal."""
                return math.sqrt(math.pow((goal_x - self.pose.x), 2) + math.pow((goal_y - self.pose.y), 2))

        def linear_vel(self, goal_x, goal_y, speed):
                return speed * self.euclidean_distance(goal_x, goal_y)

        def steering_angle(self, goal_x, goal_y):
                return math.atan2(goal_y - self.pose.y, goal_x - self.pose.x)

        def angular_vel(self, goal_x, goal_y, omega):
                return math.radians(omega) * (self.steering_angle(goal_x, goal_y) - self.pose.theta)

        def rotate_to_goal(self, goal_x, goal_y, omega):
                Kp = 1.0  # Proportional gain
                Kd = 0.5  # Derivative gain

                while abs(self.steering_angle(goal_x, goal_y) - self.pose.theta) > 0.005 and rclpy.ok():
                    error = self.steering_angle(goal_x, goal_y) - self.pose.theta
                    angular_velocity = Kp * error - Kd * self.pose.angular_velocity

                    vel_msg = Twist()
                    vel_msg.angular.z = angular_velocity
                    self.twist_pub.publish(vel_msg)
                    rclpy.spin_once(self)

                # Stop rotating
                vel_msg = Twist()
                vel_msg.angular.z = 0.0
                self.twist_pub.publish(vel_msg)
                rclpy.spin_once(self)


        def move_forward(self, goal_x, goal_y, speed):
                # Move forward towards the goal position
                vel_msg = Twist()
                vel_msg.linear.x = self.linear_vel(goal_x, goal_y, speed)

                while self.euclidean_distance(goal_x, goal_y) >= 0.1 and rclpy.ok():
                    self.twist_pub.publish(vel_msg)
                    rclpy.spin_once(self)

                # Stop moving forward
                vel_msg.linear.x = 0.0
                self.twist_pub.publish(vel_msg)
                rclpy.spin_once(self)

        def go_to(self, speed, omega, target_x, target_y):
                while self.pose is None and rclpy.ok():
                    self.get_logger().info('Waiting for pose...')
                    rclpy.spin_once(self)

                goal_x = target_x
                goal_y = target_y

                # Rotate to the goal orientation first
                self.rotate_to_goal(goal_x, goal_y, omega)

                # Move forward to the goal position
                self.move_forward(goal_x, goal_y, speed/2)


        def turn_turtle(self, target_angle, angular_speed):
                        while self.pose is None and rclpy.ok():
                            self.get_logger().info('Waiting for pose...')
                            rclpy.spin_once(self)

                        current_angle = math.degrees(self.pose.theta)
                        goal_angle = current_angle + target_angle
                        if (goal_angle>180.0):
                            goal_angle=goal_angle-180.0
                            goal_angle=-180.0+goal_angle
                        while abs(goal_angle - math.degrees(self.pose.theta)) > 0.05 and rclpy.ok():
                            error = goal_angle - math.degrees(self.pose.theta)
                            if(error>180):
                                 error=(180-error)*-1
                            if(error>0.0):
                                 angular_velocity = angular_speed*(math.log(((error/5)+(1/10))+1)+(math.sin(500*error)*0.001))
                            if(error<0.0):
                                 error=error*-1
                                 angular_velocity = -1*angular_speed*(math.log(((error/5)+(1/10))+1)+(math.sin(500*error)*0.001))

                            vel_msg = Twist()
                            vel_msg.angular.z=angular_velocity/2
                            self.twist_pub.publish(vel_msg)

                            rclpy.spin_once(self)
                            self.get_logger().info(f'Turtle Pose:theta={round(math.degrees(self.pose.theta),4)}, goal_angle={round(goal_angle,4)}, error={round(error,4)}, angular_vel={round(vel_msg.angular.z,4)}')

                            if(goal_angle<-180):
                                goal_angle=goal_angle+180.0
                                goal_angle=180.0+goal_angle

                            if (goal_angle>180.0):
                                goal_angle=goal_angle-180.0
                                goal_angle=-180.0+goal_angle

                        # Stop rotating
                        vel_msg = Twist()
                        vel_msg.angular.z = 0.0
                        self.twist_pub.publish(vel_msg)
                        rclpy.spin_once(self)
                        self.get_logger().info(f'Turtle Pose: x={self.pose.x}, y={self.pose.y}, theta={math.degrees(self.pose.theta)}')

        def move_forward(self, distance, speed):
            # Implement straght motion here
            # Create and publish msg
            vel_msg = Twist()
            if distance > 0:
                vel_msg.linear.x = speed
            else:
                vel_msg.linear.x = -speed
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

            # Set loop rate
            loop_rate = self.create_rate(100, self.get_clock()) # Hz

            # Calculate time
            T = abs(distance / speed)

            # Publish first msg and note time when to stop
            self.twist_pub.publish(vel_msg)
            #self.get_logger().info('Turtle started.')
            when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

            # Publish msg while the calculated time is up
            while (self.get_clock().now() <= when) and rclpy.ok():
                self.twist_pub.publish(vel_msg)
                #self.get_logger().info('On its way...')
                rclpy.spin_once(self)   # loop rate

            # turtle arrived, set velocity to 0
            vel_msg.linear.x = 0.0
            self.twist_pub.publish(vel_msg)
            self.get_logger().info(f'Turtle Pose: x={self.pose.x}, y={self.pose.y}, theta={math.degrees(self.pose.theta)}')


        def draw_koch_curve(self, order, size, angular_speed):
            while self.pose is None and rclpy.ok():
                self.get_logger().info('Waiting for pose...')
                rclpy.spin_once(self)

            if order == 0:
                self.move_forward(size,0.2)
            else:
                self.draw_koch_curve(order - 1, size / 3,angular_speed)
                self.turn_turtle(60,angular_speed)
                self.draw_koch_curve(order - 1, size / 3,angular_speed)
                self.turn_turtle(-120,angular_speed)
                self.draw_koch_curve(order - 1, size / 3,angular_speed)
                self.turn_turtle(60,angular_speed)
                self.draw_koch_curve(order - 1, size / 3,angular_speed)

        def draw_koch_fractal(self,order,size,angular_speed):
                for _ in range(3):
                        self.draw_koch_curve(order, size,angular_speed)
                        self.turn_turtle(-120.0,angular_speed)

def main(args=None):
    rclpy.init(args=args)
    tc = TurtlesimController()
    #tc.go_straight(1.0, 4.0)
    #tc.draw_poly(1.0, 100.0, 6, 2.0)

    tc.draw_koch_fractal(2,4,2.0)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automaticallyS
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
