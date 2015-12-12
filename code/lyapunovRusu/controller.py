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

class LyapunovController(object): #pylint: disable=too-many-instance-attributes
    """
    Configuration values for controller.
    """
    def __init__(self):
        self.x_i = 0
        self.y_i = 0
        self.yaw_i = 0
        self.x_f = 0
        self.y_f = 0
        self.yaw_f = 0
        #self.dxe = 0
        #self.dye = 0
        #self.thetastar = 0
        #self.psi = 0
        #self.e_dist = 0
        #self.thetae = 0
        #self.phi = 0
        #self.theta = 0
        #self.alpha = 0
        self.t_range = 0
        self.start = 0
        self.x_pos = []
        self.y_pos = []

    def initialize_states(self, x_i, y_i, yaw_i, x_f, y_f, yaw_f, t_end, t_inc): # pylint: disable=too-many-arguments
        """
        Initialize start and end states.
        """
        self.x_i = x_i
        self.y_i = y_i
        self.yaw_i = yaw_i
        self.x_f = x_f
        self.y_f = y_f
        self.yaw_f = yaw_f
        self.t_range = np.arange(0, t_end, t_inc) # pylint: disable=no-member

    def run(self, gamma, h_gain, k_gain, t_end, t_inc):
        """
        Run the controller.
        """
        state_init = self.calculate_errors()
        state_final = self.simulate(state_init, gamma, h_gain, k_gain)
        self.x_pos, self.y_pos = self.calculate_commands(state_final, t_end, \
            t_inc, gamma, h_gain, k_gain)

    def calculate_errors(self):
        """
        Calculate errors associated with controller state.
        """
        dxe = self.x_f - self.x_i
        dye = self.y_f - self.y_i
        thetastar = self.yaw_f
        psi = self.yaw_i
        e_dist = sqrt((dxe)**2 + (dye)**2)
        thetae = atan2(dye, dxe)
        phi = thetastar - psi
        theta = thetae - thetastar
        alpha = theta - phi
        state_init = [e_dist, theta, alpha]

        return state_init

    def simulate(self, state_init, gamma, h_gain, k_gain):
        """
        Simulate run.
        """
        self.start = time.time()
        state_final = odeint(kinematics_ode, state_init, \
            self.t_range, (gamma, h_gain, k_gain))

        return state_final

    def calculate_commands(self, state_final, t_end, t_inc, gamma, h_gain, \
        k_gain):
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
            cmd_u[t_step] = gamma * state_final[t_step, 0] * \
                cos(state_final[t_step, 1])
            cmd_w[t_step] = k_gain * state_final[t_step, 1] + \
                gamma * cos(state_final[t_step, 1]) * \
                sin(state_final[t_step, 1]) / state_final[t_step, 1] * \
                (state_final[t_step, 1] + h_gain * state_final[t_step, 2])
            x_pos[t_step] = self.x_f - state_final[t_step, 0] * \
                cos(state_final[t_step, 2])
            y_pos[t_step] = self.y_f - state_final[t_step, 0] * \
                sin(state_final[t_step, 2])
        t_elapsed = time.time() - self.start
        print 'Simulation took %0.5f seconds for a rate of %0.2f Hz.' % \
            (t_elapsed, 1 / t_elapsed)

        return x_pos, y_pos

    def plot_trajectory(self, fignum, gamma, h_gain, k_gain):
        """
        Plot trajectory of given controller.
        """
        plt.figure(fignum)
        plt.lpos, = plt.plot(self.x_pos, self.y_pos, 'b.')
        plt.lposStart, = plt.plot(self.x_i, self.y_i, 'go')
        plt.lposEnd, = plt.plot(self.x_f, self.y_f, 'ro')
        plt.title(r'Robot Trajectory ($\gamma$ = %0.2f, h = %0.2f, k = %0.2f)' \
            % (gamma, h_gain, k_gain))
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
    x_i = 5
    y_i = -2
    yaw_i = 0
    x_f = -15
    y_f = 13
    yaw_f = 0
    t_end = 40
    t_inc = 0.1
    controller.initialize_states(x_i, y_i, yaw_i, x_f, y_f, yaw_f, t_end, t_inc)
    gamma = 0.25
    h_gain = 1.2
    k_gain = 2.5
    print 'Running controller 1'
    controller.run(gamma, h_gain, k_gain, t_end, t_inc)
    controller.plot_trajectory(0, gamma, h_gain, k_gain)

    print 'Creating controller 2'
    print 'Initializing controller 2'
    controller.initialize_states(x_i, y_i, yaw_i, x_f, y_f, yaw_f, t_end, t_inc)
    gamma = 0.25
    h_gain = 1.10
    k_gain = 2 * gamma * sqrt(h_gain)
    print 'Running controller 2'
    controller.run(gamma, h_gain, k_gain, t_end, t_inc)
    controller.plot_trajectory(1, gamma, h_gain, k_gain)

    plt.show()

if __name__ == '__main__':
    main()
