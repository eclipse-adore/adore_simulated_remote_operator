#!/usr/bin/env python3

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
import pyray as rl  # Raylib
import copy

from enum import Enum
from std_msgs.msg import Bool
from std_msgs.msg import String
from adore_ros2_msgs.msg import VehicleCommand
from adore_ros2_msgs.msg import VehicleStateDynamic
from adore_ros2_msgs.msg import AssistanceRequest
from adore_ros2_msgs.msg import Map
from geometry_msgs.msg import Point

screen_width = 800
screen_height = 400
pixel_meter_ratio = 30
road_show_radius = 8  # meters

class RemoteOperationStates(Enum):
    ListeningForAssistanceRequests = 0
    CreatingWaypoints = 1
    WaitingForTrajectory = 2
    EvaluatingTrajectory = 3

class SimulatedRemoteOperatorNode(Node):
    def __init__(self):
        super().__init__('steering_display')

        # Timer for main loop
        self.timer = self.create_timer(0.02, self.run)

        # For demonstration, start at CreatingWaypoints state
        self.remote_operations_state = RemoteOperationStates.CreatingWaypoints

        self.latest_vehicle_state_dynamic = None
        self.has_received_a_vehicle_state_dynamic = False

        self.latest_local_map = None
        self.has_received_a_local_map = False

        self.latest_assitance_request = None
        self.has_received_a_new_assistance_request = False
        self.time_latest_assitance_request_was_received = 0  # Store time of last request

        self.waypoints = []

        # Subscriptions
        self.vehicle_state_dynamic_subscriber = self.create_subscription(
            VehicleStateDynamic,
            'vehicle_state/dynamic',
            self.vehicle_state_dynamic_callback,
            10
        )

        self.local_map_subscriber = self.create_subscription(
            Map,
            'local_map',
            self.local_map_callback,
            10
        )

        self.assitance_request_subscriber = self.create_subscription(
            AssistanceRequest,
            'topic_to_be_decided_1',
            self.assistance_request_callback,
            10
        )

    def run(self):
        """Called periodically by the timer; checks for assistance-request timers, etc."""
        # If no new assistance request, do nothing
        if not self.has_received_a_new_assistance_request:
            return

        # If < 30 seconds since request was received, do nothing
        now_sec = Clock().now().seconds_nanoseconds()[0]
        if (now_sec - self.time_latest_assitance_request_was_received) < 30:
            return

        # After 30 seconds, reset the flag
        self.has_received_a_new_assistance_request = False

    def publish_waypoints(self):
        # TODO: Implementation for sending waypoints
        pass

    def publish_trajectory_approval(self):
        # TODO: Implementation for approving trajectory
        pass

    def vehicle_state_dynamic_callback(self, msg):
        self.has_received_a_vehicle_state_dynamic = True
        self.latest_vehicle_state_dynamic = msg

    def local_map_callback(self, msg):
        self.has_received_a_local_map = True
        self.latest_local_map = msg

    def assistance_request_callback(self, msg):
        # Only handle a new request if not already processing one
        if self.has_received_a_new_assistance_request:
            return

        self.has_received_a_new_assistance_request = True
        self.latest_assitance_request = msg
        self.remote_operations_state = RemoteOperationStates.CreatingWaypoints
        # Record the time of this request
        self.time_latest_assitance_request_was_received = Clock().now().seconds_nanoseconds()[0]

def visualize_local_map(remote_operator_node):
    """Draw roads/lane points near the ego vehicle."""
    if not (remote_operator_node.has_received_a_local_map and 
            remote_operator_node.has_received_a_vehicle_state_dynamic):
        return

    screen_center = rl.Vector2(screen_width / 2, screen_height / 2)
    vehicle_x = remote_operator_node.latest_vehicle_state_dynamic.x
    vehicle_y = remote_operator_node.latest_vehicle_state_dynamic.y

    for road in remote_operator_node.latest_local_map.roads:
        for lane in road.lanes:
            for inner_point in lane.inner_points:
                dx = inner_point.x - vehicle_x
                dy = inner_point.y - vehicle_y
                if abs(dx) > road_show_radius or abs(dy) > road_show_radius:
                    continue

                draw_px = screen_center.x + dx * pixel_meter_ratio
                draw_py = screen_center.y - dy * pixel_meter_ratio
                rl.draw_circle(int(draw_px), int(draw_py), 5, rl.BLUE)

            for center_point in lane.center_points:
                dx = center_point.x - vehicle_x
                dy = center_point.y - vehicle_y
                if abs(dx) > road_show_radius or abs(dy) > road_show_radius:
                    continue

                draw_px = screen_center.x + dx * pixel_meter_ratio
                draw_py = screen_center.y - dy * pixel_meter_ratio
                rl.draw_circle(int(draw_px), int(draw_py), 5, rl.BLACK)

            for outer_point in lane.outer_points:
                dx = outer_point.x - vehicle_x
                dy = outer_point.y - vehicle_y
                if abs(dx) > road_show_radius or abs(dy) > road_show_radius:
                    continue

                draw_px = screen_center.x + dx * pixel_meter_ratio
                draw_py = screen_center.y - dy * pixel_meter_ratio
                rl.draw_circle(int(draw_px), int(draw_py), 5, rl.PURPLE)

def visualize_state_display(remote_operator_node):
    """Draw the textual state (e.g. Creating Waypoints, Waiting for Trajectory...)."""
    remote_operation_state_display_button_rect = rl.Rectangle(400, 10, 350, 30)
    remote_operation_state_display_button_color = rl.SKYBLUE

    rl.draw_rectangle_rec(remote_operation_state_display_button_rect, remote_operation_state_display_button_color)
    rl.draw_rectangle_lines(
        int(remote_operation_state_display_button_rect.x),
        int(remote_operation_state_display_button_rect.y),
        int(remote_operation_state_display_button_rect.width),
        int(remote_operation_state_display_button_rect.height),
        rl.GRAY
    )

    text_written_in_state = "Listening for assistance request"
    if remote_operator_node.remote_operations_state == RemoteOperationStates.CreatingWaypoints:
        text_written_in_state = "Creating Waypoints"
    elif remote_operator_node.remote_operations_state == RemoteOperationStates.WaitingForTrajectory:
        text_written_in_state = "Waiting for Trajectory"
    elif remote_operator_node.remote_operations_state == RemoteOperationStates.EvaluatingTrajectory:
        text_written_in_state = "Evaluating Trajectory"

    rl.draw_text(text_written_in_state,
                 int(remote_operation_state_display_button_rect.x) + 5,
                 int(remote_operation_state_display_button_rect.y) + 5,
                 20,
                 rl.BLACK)

def visualize_ego_vehicle(remote_operator_node):
    """Draw a simple rectangle to represent the ego vehicle."""
    if not remote_operator_node.has_received_a_vehicle_state_dynamic:
        return

    screen_center = rl.Vector2(screen_width / 2, screen_height / 2)
    car_width_pixels = pixel_meter_ratio * 1.0   # 1m wide
    car_length_pixels = pixel_meter_ratio * 2.0  # 2m long

    car_rect = rl.Rectangle(
        screen_center.x,
        screen_center.y,
        car_length_pixels,
        car_width_pixels
    )
    car_origin = rl.Vector2(car_rect.width / 2, car_rect.height / 2)
    # Convert from radians to degrees, invert sign for typical screen coords
    car_orientation_deg = -remote_operator_node.latest_vehicle_state_dynamic.yaw_angle * (180.0 / 3.14159)

    rl.draw_rectangle_pro(car_rect, car_origin, car_orientation_deg, rl.RED)

def visualize_waypoint_buttons(remote_operator_node):
    """Draw 'Reset Waypoints' and 'Send Waypoints' buttons."""
    reset_button_rect = rl.Rectangle(10, 320, 200, 30)
    reset_color = rl.SKYBLUE

    if rl.check_collision_point_rec(rl.get_mouse_position(), reset_button_rect):
        reset_color = rl.DARKBLUE

    rl.draw_rectangle_rec(reset_button_rect, reset_color)
    rl.draw_rectangle_lines(int(reset_button_rect.x), int(reset_button_rect.y),
                            int(reset_button_rect.width), int(reset_button_rect.height), rl.GRAY)

    rl.draw_text("Reset Waypoints",
                 int(reset_button_rect.x) + 5,
                 int(reset_button_rect.y) + 5,
                 20, rl.BLACK)

    send_button_rect = rl.Rectangle(10, 360, 200, 30)
    send_color = rl.SKYBLUE

    if rl.check_collision_point_rec(rl.get_mouse_position(), send_button_rect):
        send_color = rl.DARKBLUE

    rl.draw_rectangle_rec(send_button_rect, send_color)
    rl.draw_rectangle_lines(int(send_button_rect.x), int(send_button_rect.y),
                            int(send_button_rect.width), int(send_button_rect.height), rl.GRAY)

    rl.draw_text("Send Waypoints",
                 int(send_button_rect.x) + 5,
                 int(send_button_rect.y) + 5,
                 20, rl.BLACK)

def visualize_waypoints(remote_operator_node):
    """
    Draw any waypoints (in screen coords). 
    Each waypoint is a geometry_msgs/Point but *currently* 
    is stored with raw pixel x,y.
    """
    remote_operator_node.get_logger().debug(f"Waypoint count: {len(remote_operator_node.waypoints)}")

    i = 1
    for p in remote_operator_node.waypoints:
        circle_text = str(i)
        rl.draw_circle(int(p.x), int(p.y), 10, rl.RED)
        rl.draw_text(circle_text, int(p.x), int(p.y), 10, rl.WHITE)
        i += 1

def view_remote_operation_states(remote_operator_node):
    """Handle logic for different states, such as CreatingWaypoints, etc."""
    if remote_operator_node.remote_operations_state == RemoteOperationStates.CreatingWaypoints:
        # Draw waypoints on screen
        visualize_waypoints(remote_operator_node)

        # If left mouse is *down*, place a new waypoint
        if rl.is_mouse_button_down(rl.MOUSE_LEFT_BUTTON):
            mouse_position = rl.get_mouse_position()

            # Create a geometry_msgs/Point
            point_msg = Point()
            point_msg.x = float(mouse_position.x)
            point_msg.y = float(mouse_position.y)
            point_msg.z = 0.0

            remote_operator_node.waypoints.append(copy.deepcopy(point_msg))

    elif remote_operator_node.remote_operations_state == RemoteOperationStates.WaitingForTrajectory:
        pass
    elif remote_operator_node.remote_operations_state == RemoteOperationStates.EvaluatingTrajectory:
        pass

def main():
    rclpy.init()
    simulated_remote_operator_node = SimulatedRemoteOperatorNode()

    rl.init_window(screen_width, screen_height, "simulated remote operator GUI")
    rl.set_target_fps(60)

    while not rl.window_should_close():
        # Let ROS handle callbacks once per frame
        rclpy.spin_once(simulated_remote_operator_node, timeout_sec=0.0)

        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)

        # If not listening for requests, show the full UI
        if simulated_remote_operator_node.remote_operations_state != RemoteOperationStates.ListeningForAssistanceRequests:
            view_remote_operation_states(simulated_remote_operator_node)
            visualize_local_map(simulated_remote_operator_node)
            visualize_ego_vehicle(simulated_remote_operator_node)

        visualize_waypoint_buttons(simulated_remote_operator_node)
        visualize_state_display(simulated_remote_operator_node)

        rl.end_drawing()

    rl.close_window()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
