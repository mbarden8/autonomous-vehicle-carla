#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('velocity_err_previous', 0.0)
        self.vars.create_var('integral_err_previous', 0.0)
        self.vars.create_var('throttle_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """
            
            """
            Simulating how cruise control works...
            We want to track a reference velocity. We do this by splitting a high and low
            level controller. High level controller takes difference between set point
            velocity and actual vehicle velocity and generate desired acceleration to close gap.
            Low leel controller takes desired acceleration and generates a vehicle throttle
            to track reference acceleration.

            Here we will assume only throttle is needed to manage speed of vehicle (ie no braking)
            """

            # PID CONTROLLER

            # These three terms are manually tuned
            # Proportional term directly proportional to error
            K_p = 1.0
            # Integral term proportional to integral of errror
            K_i = 1.0
            # Derivative term proportional to derivative of error
            K_d = 0.01

            # Start by determining acceleration
            time_difference = t - self.vars.t_previous

            error_velocity = v_desired - v

            # Integral term
            integral_term = self.vars.integral_err_previous + error_velocity * time_difference

            # Derivative term
            derivative_term = (error_velocity - self.vars.velocity_err_previous) / 2

            # Now do PID formula to calculate new desired acceleration
            desired_acceleration = K_p * error_velocity + K_i * integral_term + K_d * derivative_term

            if desired_acceleration > 0:
                throttle_output = desired_acceleration
            else:
                throttle_output = 0


            # We do not need to worry about braking on this assignment since car will slow down on its own
            brake_output    = 0
            
            # Change the steer output with the lateral controller.

            # PURE PURSUIT CONTROLLER
            L = 2

            x_rear = x - L*np.cos(yaw)/2
            y_rear = y - L*np.sin(yaw)/2

            # # Calculate lookahead distance using middle waypoint
            lookahead_distance = np.sqrt((waypoints[len(waypoints)//2][0] - x)**2 + (waypoints[len(waypoints)//2][1] - y)**2)
            waypoint_to_target = waypoints[(len(waypoints)//2) + 1]

            distance = np.sqrt((waypoint_to_target[0] - x_rear) ** 2 + (waypoint_to_target[1] - y) ** 2)

            if distance <= lookahead_distance:
                waypoint_to_target = waypoints[0]

            # Formula for new angle to correct heading based on waypoint to target
            alpha = np.arctan2(waypoint_to_target[1] - y_rear, waypoint_to_target[0] - x_rear) - yaw

            # Formula for pure pursuit controller
            steer_output = np.arctan2(2 * L * np.sin(alpha), lookahead_distance)

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)


        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.velocity_err_previous = error_velocity
        self.vars.integral_err_previous = integral_term
        self.vars.throttle_previous = throttle_output
