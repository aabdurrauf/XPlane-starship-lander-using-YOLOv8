# -*- coding: utf-8 -*-
"""
Created on Sun Aug 18 10:56:14 2024

@author: Ammar Abdurrauf

PID controller module
reference: https://github.com/arex18/rocket-lander/blob/master/control_and_ai/pid.py
"""

import abc

class PID:
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.accumulated_error = 0
        
    def increment_intregral_error(self, error, pi_limit=3):
        self.accumulated_error = self.accumulated_error + error
        self.accumulated_error = max(min(self.accumulated_error, pi_limit), -pi_limit)

    def compute_output(self, error, dt_error):
        self.increment_intregral_error(error)
        return self.Kp * error + self.Ki * self.accumulated_error + self.Kd * dt_error

class PID_Framework:
    """ Sets the skeleton code for the actual pid algorithms (children) to inherit. """

    @abc.abstractmethod
    def pid_algorithm(self, s, x_target, y_target):
        pass
    
class PID_Benchmark(PID_Framework):
    """ Tuned PID Benchmark against which all other algorithms are compared. """

    def __init__(self):
        super(PID_Benchmark, self).__init__()
        self.Fe_PID = PID(10, 0, 10)
        self.psi_PID = PID(0.085, 0.001, 10.55)
        self.Fs_theta_PID = PID(5, 0, 6)

    def pid_algorithm(self, s, x_target=None, y_target=None):
        dx, dy, vel_x, vel_y, theta, omega, legContact_left, legContact_right = s
        if x_target is not None:
            dx = dx - x_target
        if y_target is not None:
            dy = dy - y_target
        # ------------------------------------------
        y_ref = -0.1  # Adjust speed
        y_error = y_ref - dy + 0.1 * dx
        y_dterror = -vel_y + 0.1 * vel_x

        Fe = self.Fe_PID.compute_output(y_error, y_dterror) * (abs(dx) * 50 + 1)
        # ------------------------------------------
        theta_ref = 0
        theta_error = theta_ref - theta + 0.2 * dx  # theta is negative when slanted to the north east
        theta_dterror = -omega + 0.2 * vel_x
        Fs_theta = self.Fs_theta_PID.compute_output(theta_error, theta_dterror)
        Fs = -Fs_theta  # + Fs_x
        # ------------------------------------------
        theta_ref = 0
        theta_error = -theta_ref + theta
        theta_dterror = omega
        if (abs(dx) > 0.01 and dy < 0.5):
            theta_error = theta_error - 0.06 * dx  # theta is negative when slanted to the right
            theta_dterror = theta_dterror - 0.06 * vel_x
        psi = self.psi_PID.compute_output(theta_error, theta_dterror)

        if legContact_left and legContact_right:  # legs have contact
            Fe = 0
            Fs = 0

        return Fe, Fs, psi