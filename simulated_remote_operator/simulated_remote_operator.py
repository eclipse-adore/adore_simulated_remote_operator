#!/usr/bin/env python3
import rclpy
from rclpy.clock import Clock
import pyray as rl # raylib
from rclpy.node import Node
from std_msgs.msg import String as String, Bool

from enum import Enum
from adore_ros2_msgs.msg import VehicleCommand
from adore_ros2_msgs.msg import VehicleStateDynamic
from adore_ros2_msgs.msg import AssistanceRequest
from adore_ros2_msgs.msg import Map
from geometry_msgs.msg import Point
# from adore_ros2_msgs.msg import DecisionMakerSelect
# from nav_msgs.msg import Odometry
from std_msgs.msg import String
import copy

screen_width = 800
screen_height = 400
pixel_meter_ratio = 30
road_show_radius = 8 # meters
mouse_left_click_down_detected = False # this is temporary

class SimulatedRemoteOperatorNode(Node):
    def __init__(self):
        super().__init__('steering_display')
        # self.publisher_ = self.create_publisher(VehicleStateDynamic, 'vehicle_state/dynamic', 10)
        self.timer = self.create_timer(0.002, self.run)

        # self.remote_operations_state = RemoteOperationStates.ListeningForAssistanceRequests
        self.remote_operations_state = RemoteOperationStates.CreatingWaypoints # Debug state
        
        self.latest_vehicle_state_dynamic = None
        self.has_received_a_vehicle_state_dynamic = False

        self.latest_local_map = None
        self.has_received_a_local_map = False

        self.latest_assitance_request = None
        self.has_received_a_new_assistance_request = False
        self.time_latest_assitance_request_was_received = 0

        self.waypoints = []
                
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

        self.assitance_request_subscriber = self.create_subscription(
                                                                    AssistanceRequest,
                                                                    'topic_to_be_decided_1',
                                                                    self.assistance_request_callback,
                                                                    10)

    def run(self):

        if ( self.has_received_a_new_assistance_request == False ):
            return

        if ( Clock().now().seconds_nanoseconds()[0] - self.time_latest_assitance_request_was_received < 30):
            return
            
        self.has_received_a_new_assistance_request = False

    def publish_waypoints(self):
        pass

    def publish_trajectory_approval(self):
        pass
        
    def vehicle_state_dynamic_callback(self, msg):
        if ( self.has_received_a_vehicle_state_dynamic == False ):
            self.has_received_a_vehicle_state_dynamic = True
        self.latest_vehicle_state_dynamic = msg

    def local_map_callback(self, msg):
        if ( self.has_received_a_local_map == False ):
            self.has_received_a_local_map = True

        self.latest_local_map = msg

    def local_map_callback(self, msg):
        if ( self.has_received_a_local_map == False ):
            self.has_received_a_local_map = True

        self.latest_local_map = msg

    def assistance_request_callback(self, msg):

        if ( self.has_received_a_new_assistance_request == True ):
            return

        if (self.has_received_a_new_assistance_request == False):
            self.has_received_a_new_assistance_request = True

        self.latest_assitance_request = msg
        self.remote_operations_state = RemoteOperationStates.CreatingWaypoints

class RemoteOperationStates(Enum):
    ListeningForAssistanceRequests = 0
    CreatingWaypoints = 1
    WaitingForTrajectory = 2
    EvaluatingTrajectory = 3

def visualize_local_map(remote_operator_node):

    if ( remote_operator_node.has_received_a_local_map == False or remote_operator_node.has_received_a_vehicle_state_dynamic == False):
        return

    screen_center = rl.Vector2(screen_width / 2, screen_height / 2)
    for road in remote_operator_node.latest_local_map.roads:
            # pass
        for lane in road.lanes:
            for inner_point in lane.inner_points:

                utm_px = inner_point.x
                utm_py = inner_point.y

                delta_pos_x_to_vehicle = inner_point.x - remote_operator_node.latest_vehicle_state_dynamic.x
                delta_pos_y_to_vehicle = inner_point.y - remote_operator_node.latest_vehicle_state_dynamic.y
                if ( delta_pos_x_to_vehicle > road_show_radius  or  delta_pos_x_to_vehicle < -road_show_radius  or delta_pos_y_to_vehicle > road_show_radius  or delta_pos_y_to_vehicle < -road_show_radius ):
                    continue

                draw_px = screen_center.x + delta_pos_x_to_vehicle * pixel_meter_ratio
                draw_py = screen_center.y - delta_pos_y_to_vehicle * pixel_meter_ratio
                    # simulated_remote_operator_node.get_logger().info('Position: "%s"' % delta_pos_x_to_vehicle)
                rl.draw_circle(int(draw_px), int(draw_py), 5, rl.BLUE)

            for center_point in lane.center_points:

                utm_px = center_point.x
                utm_py = center_point.y

                delta_pos_x_to_vehicle = center_point.x - remote_operator_node.latest_vehicle_state_dynamic.x
                delta_pos_y_to_vehicle = center_point.y - remote_operator_node.latest_vehicle_state_dynamic.y

                if ( delta_pos_x_to_vehicle > road_show_radius or  delta_pos_x_to_vehicle < -road_show_radius or delta_pos_y_to_vehicle > road_show_radius or delta_pos_y_to_vehicle < -road_show_radius) :
                    continue

                draw_px = screen_center.x + delta_pos_x_to_vehicle * pixel_meter_ratio
                draw_py = screen_center.y - delta_pos_y_to_vehicle * pixel_meter_ratio
                    # simulated_remote_operator_node.get_logger().info('Position: "%s"' % delta_pos_x_to_vehicle)
                rl.draw_circle(int(draw_px), int(draw_py), 5, rl.BLACK)

            for outer_point in lane.outer_points:

                utm_px = outer_point.x
                utm_py = outer_point.y

                delta_pos_x_to_vehicle = outer_point.x - remote_operator_node.latest_vehicle_state_dynamic.x
                delta_pos_y_to_vehicle = outer_point.y - remote_operator_node.latest_vehicle_state_dynamic.y

                if ( delta_pos_x_to_vehicle > road_show_radius or  delta_pos_x_to_vehicle < -road_show_radius or delta_pos_y_to_vehicle > road_show_radius or delta_pos_y_to_vehicle < -road_show_radius ):
                    continue

                draw_px = screen_center.x + delta_pos_x_to_vehicle * pixel_meter_ratio
                draw_py = screen_center.y - delta_pos_y_to_vehicle * pixel_meter_ratio
                    # simulated_remote_operator_node.get_logger().info('Position: "%s"' % delta_pos_x_to_vehicle)

                rl.draw_circle(int(draw_px), int(draw_py), 5, rl.PURPLE)

def visualize_state_display(remote_operator_node):

    remote_operation_state_display_button_rect = rl.Rectangle(400, 10, 350, 30)
    remote_operation_state_display_button_color = rl.SKYBLUE

    rl.draw_rectangle_rec(remote_operation_state_display_button_rect, remote_operation_state_display_button_color)
    rl.draw_rectangle_lines(int(remote_operation_state_display_button_rect.x), int(remote_operation_state_display_button_rect.y), int(remote_operation_state_display_button_rect.width), int(remote_operation_state_display_button_rect.height), rl.GRAY)

    text_written_in_state = "Listening for assistance request"

    # simulated_remote_operator_node.get_logger().info('Position: "%s"' % delta_pos_x_to_vehicle)
    # remote_operator_node.get_logger().info(f"State: {remote_operator_node.remote_operations_state}")
    if ( remote_operator_node.remote_operations_state is RemoteOperationStates.CreatingWaypoints ):
        # remote_operator_node.get_logger().info("Entered here")
        text_written_in_state = "Creating Waypoint"

    if ( remote_operator_node.remote_operations_state is RemoteOperationStates.WaitingForTrajectory ):
        text_written_in_state = "Waiting for Trajectory"

    if ( remote_operator_node.remote_operations_state is RemoteOperationStates.EvaluatingTrajectory ):
        text_written_in_state = "Evaluating Trajectory"

    rl.draw_text(text_written_in_state, int(remote_operation_state_display_button_rect.x) + 5, int(remote_operation_state_display_button_rect.y) + 5, 20, rl.BLACK)

def visualize_ego_vehicle(remote_operator_node):

    screen_center = rl.Vector2(screen_width / 2, screen_height / 2)

    if ( remote_operator_node.has_received_a_vehicle_state_dynamic == False ):
        return

    car_width = pixel_meter_ratio * 1 # meters to pixel
    car_length = pixel_meter_ratio * 2

    car_rectangle = rl.Rectangle(screen_center.x, screen_center.y, int(car_length), int(car_width))
    car_origin = rl.Vector2(car_rectangle.width / 2, car_rectangle.height / 2)
    car_orientation = -remote_operator_node.latest_vehicle_state_dynamic.yaw_angle * (180 / 3.14159)

    rl.draw_rectangle_pro(car_rectangle, car_origin, car_orientation, rl.RED)

def visualize_waypoint_buttons(remote_operator_node):

    reset_waypoint_button_rect = rl.Rectangle(10, 320, 200, 30)
    reset_waypoint_button_color = rl.SKYBLUE

    if ( rl.check_collision_point_rec(rl.get_mouse_position(), reset_waypoint_button_rect) ):
        reset_waypoint_button_color = rl.DARKBLUE

    rl.draw_rectangle_rec(reset_waypoint_button_rect, reset_waypoint_button_color)
    rl.draw_rectangle_lines(int(reset_waypoint_button_rect.x), int(reset_waypoint_button_rect.y), int(reset_waypoint_button_rect.width), int(reset_waypoint_button_rect.height), rl.GRAY)


    reset_waypoints_text = "Reset Waypoints"
    rl.draw_text(reset_waypoints_text, int(reset_waypoint_button_rect.x) + 5, int(reset_waypoint_button_rect.y) + 5, 20, rl.BLACK)

    send_waypoints_button_rect = rl.Rectangle(10, 360, 200, 30)
    send_waypoints_button_color = rl.SKYBLUE

    if ( rl.check_collision_point_rec(rl.get_mouse_position(), send_waypoints_button_rect) ):
        send_waypoints_button_color = rl.DARKBLUE

    rl.draw_rectangle_rec(send_waypoints_button_rect , send_waypoints_button_color)
    rl.draw_rectangle_lines(int(send_waypoints_button_rect.x), int(send_waypoints_button_rect.y), int(send_waypoints_button_rect.width), int(send_waypoints_button_rect.height), rl.GRAY)

    send_waypoints_text = "Send Waypoints"
    rl.draw_text(send_waypoints_text, int(send_waypoints_button_rect.x) + 5, int(send_waypoints_button_rect.y) + 5, 20, rl.BLACK)
    return

    remote_operation_state_display_button_rect = rl.Rectangle(400, 10, 350, 30)
    remote_operation_state_display_button_color = rl.SKYBLUE

    rl.draw_rectangle_rec(remote_operation_state_display_button_rect, remote_operation_state_display_button_color)
    rl.draw_rectangle_lines(int(remote_operation_state_display_button_rect.x), int(remote_operation_state_display_button_rect.y), int(remote_operation_state_display_button_rect.width), int(remote_operation_state_display_button_rect.height), rl.GRAY)

    button = rl.Rectangle(100, 100, 100, 100)

    button_color = rl.SKYBLUE
    if ( rl.check_collision_point_rec(rl.get_mouse_position(), button) ):
        button_color = rl.DARKBLUE

    rl.draw_rectangle_rec(button, button_color)
    rl.draw_rectangle_lines(int(button.x), int(button.y), int(button.width), int(button.height), rl.GRAY)

def view_remote_operation_states(remote_operator_node):

    if ( remote_operator_node.remote_operations_state is RemoteOperationStates.CreatingWaypoints ):


        visualize_waypoints(remote_operator_node)

        
        # global mouse_left_click_down_detected
        if ( rl.is_mouse_button_down(rl.MOUSE_LEFT_BUTTON) == False ):
            return



        # if ( rl.is_mouse_button_up(rl.MOUSE_LEFT_BUTTON) == True and mouse_left_click_down_detected == True):
            # pass
        point_msg = Point
        mouse_position = rl.get_mouse_position()
        point_msg.x = mouse_position.x
        point_msg.y = mouse_position.y
        remote_operator_node.waypoints.append(copy.deepcopy(point_msg))
        # mouse_left_click_down_detected = False
        # remote_operator_node.get_logger().info("Placed waypoint")

    if ( remote_operator_node.remote_operations_state is RemoteOperationStates.WaitingForTrajectory ):
        pass
    
    if ( remote_operator_node.remote_operations_state is RemoteOperationStates.EvaluatingTrajectory ):
        pass

def visualize_waypoints(remote_operator_node):

    remote_operator_node.get_logger().info(f"Waypoint length : {len(remote_operator_node.waypoints)}")
    i = 0

    # for i in range(0, len(remote_operator_node.waypoints)):

        # remote_operator_node.get_logger().info(f"Point position : {remote_operator_node.waypoints[i].x}")

    for p in remote_operator_node.waypoints:
        i += 1
        circle_text = str(i)
        rl.draw_circle(int(p.x), int(p.y), 10, rl.RED)
        rl.draw_text(circle_text, int(p.x), int(p.y), 10, rl.WHITE)


def main():
    rclpy.init()
    simulated_remote_operator_node = SimulatedRemoteOperatorNode()

    rl.init_window(screen_width, screen_height, "simulated remote operator GUI")
    rl.set_target_fps(60)

    while not rl.window_should_close():
        rclpy.spin_once( simulated_remote_operator_node )

        rl.begin_drawing()

        if ( simulated_remote_operator_node.remote_operations_state != RemoteOperationStates.ListeningForAssistanceRequests):
            
            view_remote_operation_states(simulated_remote_operator_node)
            visualize_local_map(simulated_remote_operator_node)
            visualize_ego_vehicle(simulated_remote_operator_node)

        visualize_waypoint_buttons(simulated_remote_operator_node)
        visualize_state_display(simulated_remote_operator_node)
        rl.clear_background(rl.RAYWHITE)

        # rl.end_drawing()

    rl.close_window()

if __name__ == '__main__':
    main()
