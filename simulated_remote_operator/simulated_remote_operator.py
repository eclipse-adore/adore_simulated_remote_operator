#!/usr/bin/env python3
import rclpy
import pyray as rl # raylib
from rclpy.node import Node
from std_msgs.msg import String as String, Bool

from adore_ros2_msgs.msg import VehicleCommand
from adore_ros2_msgs.msg import VehicleStateDynamic
from adore_ros2_msgs.msg import Map
# from adore_ros2_msgs.msg import DecisionMakerSelect
# from nav_msgs.msg import Odometry
from std_msgs.msg import String


class SimulatedRemoteOperatorNode(Node):
    def __init__(self):
        super().__init__('steering_display')
        # self.publisher_ = self.create_publisher(VehicleStateDynamic, 'vehicle_state/dynamic', 10)
        self.timer = self.create_timer(0.002, self.run)
        self.latest_vehicle_state_dynamic = None
        self.has_received_a_vehicle_state_dynamic = False
        self.latest_local_map = None
        self.has_received_a_local_map = False
        self.vehicle_state_dynamic_subscriber = self.create_subscription(
                                                    VehicleStateDynamic,
                                                    'vehicle_state/dynamic',
                                                    self.vehicle_state_dynamic_callback,
                                                    10)

        self.local_map_subscriber = self.create_subscription(
                                                    Map,
                                                    'local_map',
                                                    self.local_map_callback,
                                                    10)


    def run(self):

        if ( self.has_received_a_vehicle_state_dynamic == False ):
            return
        
        self.get_logger().info('Publishing: "%s"' % 0)

    def vehicle_state_dynamic_callback(self, msg):
        if ( self.has_received_a_vehicle_state_dynamic == False ):
            self.has_received_a_vehicle_state_dynamic = True
        self.latest_vehicle_state_dynamic = msg

    def local_map_callback(self, msg):
        if ( self.has_received_a_local_map == False ):
            self.has_received_a_local_map = True

        self.latest_local_map = msg

def main():
    rclpy.init()
    simulated_remote_operator_node = SimulatedRemoteOperatorNode()

    screen_width = 800;
    screen_height = 400;

    rl.init_window(screen_width, screen_height, "simulated remote operator GUI")
    rl.set_target_fps(60)

    while not rl.window_should_close():
        rclpy.spin_once(simulated_remote_operator_node )

        rl.begin_drawing()

        screen_center = rl.Vector2(screen_width / 2, screen_height / 2)


        if ( simulated_remote_operator_node.has_received_a_local_map == True and simulated_remote_operator_node.has_received_a_vehicle_state_dynamic == True):
            for road in simulated_remote_operator_node.latest_local_map.roads:
                # pass
                for lane in road.lanes:
                    for inner_point in lane.inner_points:

                        utm_px = inner_point.x
                        utm_py = inner_point.y

                        delta_pos_x_to_vehicle = inner_point.x - simulated_remote_operator_node.latest_vehicle_state_dynamic.x
                        delta_pos_y_to_vehicle = inner_point.y - simulated_remote_operator_node.latest_vehicle_state_dynamic.y


                        if ( delta_pos_x_to_vehicle > 8 or  delta_pos_x_to_vehicle < -8 or delta_pos_y_to_vehicle > 8 or delta_pos_y_to_vehicle < -8):
                            continue
                    
                        draw_px = screen_center.x - delta_pos_x_to_vehicle * 50
                        draw_py = screen_center.y - delta_pos_y_to_vehicle * 50

                        # simulated_remote_operator_node.get_logger().info('Position: "%s"' % delta_pos_x_to_vehicle)

                        rl.draw_circle(int(draw_px), int(draw_py), 5, rl.BLUE)

                    for center_point in lane.center_points:

                        utm_px = center_point.x
                        utm_py = center_point.y

                        delta_pos_x_to_vehicle = center_point.x - simulated_remote_operator_node.latest_vehicle_state_dynamic.x
                        delta_pos_y_to_vehicle = center_point.y - simulated_remote_operator_node.latest_vehicle_state_dynamic.y


                        if ( delta_pos_x_to_vehicle > 8 or  delta_pos_x_to_vehicle < -8 or delta_pos_y_to_vehicle > 8 or delta_pos_y_to_vehicle < -8) :
                            continue
                    
                        draw_px = screen_center.x - delta_pos_x_to_vehicle * 50
                        draw_py = screen_center.y - delta_pos_y_to_vehicle * 50

                        # simulated_remote_operator_node.get_logger().info('Position: "%s"' % delta_pos_x_to_vehicle)

                        rl.draw_circle(int(draw_px), int(draw_py), 5, rl.BLACK)

                    for outer_point in lane.outer_points:

                        utm_px = outer_point.x
                        utm_py = outer_point.y

                        delta_pos_x_to_vehicle = outer_point.x - simulated_remote_operator_node.latest_vehicle_state_dynamic.x
                        delta_pos_y_to_vehicle = outer_point.y - simulated_remote_operator_node.latest_vehicle_state_dynamic.y


                        if ( delta_pos_x_to_vehicle > 8 or  delta_pos_x_to_vehicle < -8 or delta_pos_y_to_vehicle > 8 or delta_pos_y_to_vehicle < -8 ):
                            continue
                    
                        draw_px = screen_center.x - delta_pos_x_to_vehicle * 50
                        draw_py = screen_center.y - delta_pos_y_to_vehicle * 50

                        # simulated_remote_operator_node.get_logger().info('Position: "%s"' % delta_pos_x_to_vehicle)

                        rl.draw_circle(int(draw_px), int(draw_py), 5, rl.PURPLE)



        if ( simulated_remote_operator_node.has_received_a_vehicle_state_dynamic == True ):

            car_rectangle = rl.Rectangle(screen_center.x, screen_center.y, 100, 50)
            car_origin = rl.Vector2(car_rectangle.width / 2, car_rectangle.height / 2)
            car_orientation = -simulated_remote_operator_node.latest_vehicle_state_dynamic.yaw_angle * (180 / 3.14159)

            rl.draw_rectangle_pro(car_rectangle, car_origin, car_orientation, rl.RED) 

        rl.clear_background(rl.RAYWHITE)

        rl.end_drawing()

    rl.close_window()


if __name__ == '__main__':
    main()
