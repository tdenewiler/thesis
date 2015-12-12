#! /usr/bin/env python
"""
Simulation of differential drive robot using model-based controller based on
Lyapunov stability theory.
"""

import matplotlib
# For use over SSH.
matplotlib.use('Qt4Agg')

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint # pylint: disable=no-name-in-module
from math import sqrt, atan2, cos, sin
import time

PNGRES = 100
SAVEIMAGES = 0

def kinematics_ode(state, dummy, gamma, h_gain, k_gain):
    r"""
    Solves the differential equations from robot kinematics equations to
    generate a trajectory that the robot will drive.

    ODEs
    .. math::
        \dot{e} = -\gamma e \cos^2\alpha
        \dot{\alpha} = -k\alpha - \gamma h \frac{\cos\alpha\sin\alpha}{\alpha}
        \dot{\theta} = \gamma\cos\alpha\sin\alpha

    Variables:
    state(0) = e
    state(1) = alpha
    state(2) = theta
    """
    edot = -gamma * state[0] * cos(state[1])**2
    alphadot = -k_gain * state[1] - \
        gamma * h_gain * state[2] * cos(state[1]) * sin(state[1]) / state[1]
    thetadot = gamma * cos(state[1]) * sin(state[1])

    return [edot, alphadot, thetadot]

class LyapunovController(object):
    """
    Configuration values for controller.
    """
    def __init__(self):
        self.pose_init = [0, 0, 0] # (x, y, yaw)
        self.pose_final = [0, 0, 0] # (x, y, yaw)
        self.t_range = 0
        self.start = 0
        self.x_pos = []
        self.y_pos = []

    def initialize_states(self, pose_init, pose_final, t_end, t_inc):
        """
        Initialize start and end states.
        """
        self.pose_init = pose_init
        self.pose_final = pose_final
        self.t_range = np.arange(0, t_end, t_inc) # pylint: disable=no-member

    def run(self, gains, t_end, t_inc):
        """
        Run the controller.
        """
        state_init = self.calculate_errors()
        state_final = self.simulate(state_init, gains)
        self.x_pos, self.y_pos = self.calculate_commands(state_final, t_end, \
            t_inc, gains)

    def calculate_errors(self):
        """
        Calculate errors associated with controller state.
        """
        dxe = self.pose_final[0] - self.pose_init[0]
        dye = self.pose_final[1] - self.pose_init[1]
        thetastar = self.pose_final[2]
        psi = self.pose_init[2]
        e_dist = sqrt((dxe)**2 + (dye)**2)
        thetae = atan2(dye, dxe)
        phi = thetastar - psi
        theta = thetae - thetastar
        alpha = theta - phi
        state_init = [e_dist, theta, alpha]

        return state_init

    def simulate(self, state_init, gains):
        """
        Simulate run.
        """
        self.start = time.time()
        state_final = odeint(kinematics_ode, state_init, \
            self.t_range, (gains[0], gains[1], gains[2]))

        return state_final

    def calculate_commands(self, state_final, t_end, t_inc, gains):
        """
        Calculate commands at each time step.
        """
# pylint: disable=no-member
        x_pos = np.zeros((t_end / t_inc, 1))
        y_pos = np.zeros((t_end / t_inc, 1))
        cmd_u = np.zeros((t_end / t_inc, 1))
        cmd_w = np.zeros((t_end / t_inc, 1))
# pylint: enable=no-member

        for t_step in range(0, int(t_end / t_inc)):
            cmd_u[t_step] = gains[0] * state_final[t_step, 0] * \
                cos(state_final[t_step, 1])
            cmd_w[t_step] = gains[2] * state_final[t_step, 1] + \
                gains[0] * cos(state_final[t_step, 1]) * \
                sin(state_final[t_step, 1]) / state_final[t_step, 1] * \
                (state_final[t_step, 1] + gains[1] * state_final[t_step, 2])
            x_pos[t_step] = self.pose_final[0] - state_final[t_step, 0] * \
                cos(state_final[t_step, 2])
            y_pos[t_step] = self.pose_final[1] - state_final[t_step, 0] * \
                sin(state_final[t_step, 2])
        t_elapsed = time.time() - self.start
        print 'Simulation took %0.5f seconds for a rate of %0.2f Hz.' % \
            (t_elapsed, 1 / t_elapsed)

        return x_pos, y_pos

    def plot_trajectory(self, fignum, gains):
        """
        Plot trajectory of given controller.
        """
        plt.figure(fignum)
        plt.lpos, = plt.plot(self.x_pos, self.y_pos, 'b.')
        plt.lposStart, = plt.plot(self.pose_init[0], self.pose_init[1], 'go')
        plt.lposEnd, = plt.plot(self.pose_final[0], self.pose_final[1], 'ro')
        plt.title(r'Robot Trajectory ($\gamma$ = %0.2f, h = %0.2f, k = %0.2f)' \
            % (gains[0], gains[1], gains[2]))
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend((plt.lpos, plt.lposStart, plt.lposEnd), ('Position', \
            'Start', 'End'), 'best')
        plt.axis('equal')
        if SAVEIMAGES:
            plt.savefig("images/lyapunovTrajectory.png", dpi=PNGRES)

def main():
    """
    Run controllers with different gains.
    """
    print 'Creating controller 1'
    controller = LyapunovController()
    print 'Initializing controller 1'
    pose_init = [5, -2, 0]
    pose_final = [-15, 13, 0]
    t_end = 40
    t_inc = 0.1
    controller.initialize_states(pose_init, pose_final, t_end, t_inc)
    gains = [0.25, 1.2, 2.5] # [gamma, h, k]
    print 'Running controller 1'
    controller.run(gains, t_end, t_inc)
    controller.plot_trajectory(0, gains)

    print 'Creating controller 2'
    print 'Initializing controller 2'
    controller.initialize_states(pose_init, pose_final, t_end, t_inc)
    gains = [0.25, 1.2, 2.5] # [gamma, h, k]
    print 'Running controller 2'
    controller.run(gains, t_end, t_inc)
    controller.plot_trajectory(1, gains)

    plt.show()

if __name__ == '__main__':
    main()
