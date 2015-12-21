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
import sys
from optparse import OptionParser

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
        parser = OptionParser()
        parser.add_option("-r", "--resolution", dest="resolution", \
            help="Resolution for output images of plots.", default=100)
        parser.add_option("-s", "--save-images", dest="save_images", \
            help="Whether to save images or not.", default=False)
        options, dummy = parser.parse_args(sys.argv)

        self.positions = [[], []]
        self.pose_init = [0, 0, 0] # (x, y, yaw)
        self.pose_final = [0, 0, 0] # (x, y, yaw)
        self.start = time.time()
        self.x_pos = []
        self.y_pos = []
        self.image_props = [options.resolution, options.save_images]

    def initialize_states(self, pose_init, pose_final):
        """
        Initialize start and end states.
        """
        self.pose_init = pose_init
        self.pose_final = pose_final

    def run(self, gains, t_end, t_inc):
        """
        Run the controller.
        """
        for trial in range(len(gains)):
            state_init = self.calculate_errors()
            state_final = self.simulate(state_init, gains[trial], t_end, t_inc)
            self.calculate_commands(state_final, t_end, t_inc, gains[trial])

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

    @classmethod
    def simulate(cls, state_init, gains, t_end, t_inc):
        """
        Simulate run.
        """
        t_range = np.arange(0, t_end, t_inc) # pylint: disable=no-member
        state_final = odeint(kinematics_ode, state_init, \
            t_range, (gains[0], gains[1], gains[2]))

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

        self.x_pos.append(x_pos)
        self.y_pos.append(y_pos)

    def plot_trajectory(self, gains):
        """
        Plot trajectory of given controller.
        """
        plt.figure()
#pylint: disable=no-member
        colormap = plt.cm.gist_rainbow
        plt.gca().set_color_cycle([colormap(i) for i in np.linspace(0, 1, \
            len(gains))])
#pylint: enable=no-member
        labels = []
        for entry in range(len(gains)):
            plt.lpos, = plt.plot(self.x_pos[entry], self.y_pos[entry])
            labels.append(r'(%0.2lf, %0.2lf, %0.2lf)' % (gains[entry][0], \
                gains[entry][1], gains[entry][2]))
        plt.lpos_start, = plt.plot(self.pose_init[0], self.pose_init[1])
        plt.lpos_end, = plt.plot(self.pose_final[0], self.pose_final[1])
        plt.title(r'Robot Trajectory')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        labels.append('Start')
        labels.append('End')
        #plt.legend(labels, ncol=4, loc='best')
        plt.axis('equal')
        if self.image_props[1]:
            name = "images/lyapunovTrajectories.png"
            plt.savefig(name, dpi=self.image_props[0])

def main():
    """
    Run controllers with different gains.
    """
    controller = LyapunovController()
    pose_init = [5, -2, 0] # [x, y, yaw]
    pose_final = [-15, 13, 0]
    pose_init = [0, 0, 0] # [x, y, yaw]
    pose_final = [10, 10, 0]
    num_trials = 100
    t_end = 30
    t_inc = 0.1
    k_gamma = 0.1
    k_h = 1.0
    k_k = k_gamma * sqrt(k_h)
    controller.initialize_states(pose_init, pose_final)
    gains = []
    for trial in range(num_trials):
        k_gamma = k_gamma + trial * 0.01
        k_h = k_h + trial * 0.01
        k_k = k_gamma * sqrt(k_h)
        gains.append([k_gamma, k_h, k_k])
    controller.run(gains, t_end, t_inc)
    t_elapsed = time.time() - controller.start
    print 'Simulation of %d runs took %0.5f seconds for a rate of %0.2f Hz.' % \
        (num_trials, t_elapsed, 1 / t_elapsed)
    controller.plot_trajectory(gains)

    plt.show()

if __name__ == '__main__':
    main()
