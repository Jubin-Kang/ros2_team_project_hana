import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from .hanalib import Motor

from rcl_interfaces.msg import SetParametersResult
import time

class hanaBringup(Node):
 
    def __init__(self):
        super().__init__('hana_bringup')
 
        self.hana = Motor()

        self.hana.enable_motor()
        self.hana.start_motor()
 
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            1
        )

        self.declare_parameter('motor_ratio', 1.0) # 왼쪽 모터 출력 비율 설정
        self.motor_ratio = self.get_parameter('motor_ratio').value
        self.hana.set_ratio(self.motor_ratio)
     
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("hana is ready!!")

 
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'motor_ratio':
                self.motor_ratio = param.value
                self.hana.set_ratio(self.motor_ratio)

        self.get_logger().info(f"set L motor ratio {self.motor_ratio * 100} %")
        
        return SetParametersResult(successful=True)
 
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x 
        angular_z = msg.angular.z/5

        # 좌우 회전
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        set_l = self.custom_map(left_speed)
        set_r = self.custom_map(right_speed)
 
        self.hana.move(set_l*0.75, set_r*0.75)
        self.cnt = 0

    def custom_map(self, value):
        if value == 0:
            return 0
        elif value > 0:
            result = 25 + ((value * 20)/1) #25
        else:
            result = -25 + ((value * 20)/1)
        
        return max(min(result, 100), -100)

    def destroy_node(self):
        self.hana.disable_motor()
        self.hana.stop_motor()
        self.hana.clean()
        
def main(args=None):
    rclpy.init(args=args)
    hana_bringup_node = hanaBringup()
     
    try:
        rclpy.spin(hana_bringup_node)
    except KeyboardInterrupt:
        pass
    finally:
        hana_bringup_node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
