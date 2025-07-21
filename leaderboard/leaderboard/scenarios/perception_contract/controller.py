from typing import Tuple, List
import math
import numpy as np
from collections import deque


class PIDController(object):
    def __init__(self, K_P=1.0, K_I=0.0, K_D=0.0, n=20):
        self._K_P = K_P
        self._K_I = K_I
        self._K_D = K_D

        self._window = deque([0 for _ in range(n)], maxlen=n)

    def step(self, error):
        self._window.append(error)

        if len(self._window) >= 2:
            integral = np.mean(self._window)
            derivative = (self._window[-1] - self._window[-2])
        else:
            integral = 0.0
            derivative = 0.0

        return self._K_P * error + self._K_I * integral + self._K_D * derivative

turn_KP = 1.25
turn_KI = 0.75
turn_KD = 0.3
turn_n = 20
turn_controller = PIDController(K_P=turn_KP, K_I=turn_KI, K_D=turn_KD, n=turn_n)

speed_KP = 5.0
speed_KI = 0.5
speed_KD = 1.0
speed_n = 20 
speed_controller = PIDController(K_P=speed_KP, K_I=speed_KI, K_D=speed_KD, n=speed_n)

def control_pid(waypoints:np.ndarray, speed:np.ndarray):
    ''' Predicts vehicle control with a PID controller.
    Args:
        waypoints (tensor): output of self.plan()
        velocity (tensor): speedometer input
    '''

    desired_speed = np.linalg.norm(waypoints[0] - waypoints[1]) * 2.0

    # if is_stuck:
    #     desired_speed = np.array(4) # default speed of 14.4 km/h

    brake = ((desired_speed < 0.4) or ((speed / desired_speed) > 1.1))

    delta = np.clip(desired_speed - speed, 0.0, 0.25)
    throttle = speed_controller.step(delta)
    throttle = np.clip(throttle, 0.0, 0.75)
    throttle = throttle if not brake else 0.0
    aim = (waypoints[1] + waypoints[0]) / 2.0
    angle = np.degrees(np.arctan2(aim[1], aim[0])) / 90.0
    if (speed < 0.01):
        angle = 0.0  # When we don't move we don't want the angle error to accumulate in the integral
    if brake:
        angle = 0.0
    
    steer = turn_controller.step(angle)

    steer = np.clip(steer, -1.0, 1.0) #Valid steering values are in [-1,1]

    return steer, throttle, brake


class EgoModel():
    def __init__(self, dt=1./20.0):
        self.dt = dt
        
        # Kinematic bicycle model. Numbers are the tuned parameters from World on Rails
        self.front_wb    = -0.090769015
        self.rear_wb     = 1.4178275

        self.steer_gain  = 0.36848336
        self.brake_accel = -4.952399
        self.throt_accel = 0.5633837

    def forward(self, locs, yaws, spds, acts) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Kinematic bicycle model. Numbers are the tuned parameters from World on Rails
        steer = acts[..., 0:1].item()
        throt = acts[..., 1:2].item()
        brake = acts[..., 2:3].astype(np.uint8)

        if (brake):
            accel = self.brake_accel
        else:
            accel = self.throt_accel * throt

        wheel = self.steer_gain * steer

        beta = math.atan(self.rear_wb / (self.front_wb + self.rear_wb) * math.tan(wheel))
        yaws = yaws.item()
        spds = spds.item()
        next_locs_0 = locs[0].item() + spds * math.cos(yaws + beta) * self.dt
        next_locs_1 = locs[1].item() + spds * math.sin(yaws + beta) * self.dt
        next_yaws = yaws + spds / self.rear_wb * math.sin(beta) * self.dt
        next_spds = spds + accel * self.dt
        next_spds = next_spds * (next_spds > 0.0)  # Fast ReLU

        next_locs = np.array([next_locs_0, next_locs_1])
        next_yaws = np.array(next_yaws)
        next_spds = np.array(next_spds)

        return next_locs, next_yaws, next_spds
    
    def value_at(self, eqn, t):
        """
        Evaluate the polynomial equation at time t.
        Args:
            eqn (np.ndarray): Polynomial coefficients for x, y, yaw, and speed.
            t (float): Time at which to evaluate the polynomial.
        Returns:
            value (float): Evaluated value of the polynomial at time t.
        """
        eqn = np.flipud(eqn)  # Flip the coefficients for np.polyval
        return np.polyval(eqn, t)  
    
    def forward_continuous(self, locs, yaws, spds, acts):
        """
        Forward method for continuous control.
        Args:
            locs (np.ndarray): Current coordinates of the ego vehicle.
            yaws (np.ndarray): Current yaw angle of the ego vehicle.
            spds (np.ndarray): Current speed of the ego vehicle.
            acts (np.ndarray): Actions to be applied.
        Returns:
            eqns (np.ndarray): Array of polynomial coefficients for x, y, yaw, and speed.
        """
        steer = acts[..., 0:1].item()
        throt = acts[..., 1:2].item()
        brake = acts[..., 2:3].astype(np.uint8)
        if (brake):
            accel = self.brake_accel
        else:
            accel = self.throt_accel * throt
        wheel = self.steer_gain * steer
        beta = math.atan(self.rear_wb / (self.front_wb + self.rear_wb) * math.tan(wheel))
        yaws = yaws.item()
        spds = spds.item()
        x_init = locs[0].item()
        y_init = locs[1].item()
        yaw_init = yaws
        speed_init = spds

        # Calculate the coefficients for the polynomial equations
        x_coeff = [
            x_init,
            speed_init * math.cos(yaw_init + beta),
            0.5 * accel * math.cos(yaw_init + beta),
            # 0.0
        ]
        y_coeff = [
            y_init,
            speed_init * math.sin(yaw_init + beta),
            0.5 * accel * math.sin(yaw_init + beta),
            # 0.0
        ]
        yaw_coeff = [
            yaw_init,
            speed_init / self.rear_wb * math.sin(beta),
            0.0,
            # 0.0
        ]
        speed_coeff = [
            speed_init,
            accel,
            0.0,
            # 0.0
        ]
        eqns = np.array([x_coeff, y_coeff, yaw_coeff, speed_coeff])
        return eqns
