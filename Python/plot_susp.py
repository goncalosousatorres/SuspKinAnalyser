import matplotlib.pyplot as plt
import numpy as np
import math


def plot_body(ax, body, linefrmt, clr, close=1):
    if close == 1 and body.shape[1] > 2:
        body = np.append(body, body[:, 0].reshape(3, 1), axis=1)

    ax.plot(body[0, :], body[1, :], body[2, :], linefrmt, color=clr)


def plot_susp(HP, virtual=False, mirror=False):


    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.view_init(0, 0, 0)
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    if mirror is True:
        i = [1, -1]
    else:
        i = [1]

    for kx in i:

        HP[1, :] = kx * HP[1, :]

        # Upper A-Arm
        b1 = HP[:, :3]
        plot_body(ax, b1, 'o-', 'red')

        # Lower A-Arm
        b2 = HP[:, 3:6]
        plot_body(ax, b2, 'o-', 'green')

        # Upright
        b3 = HP[:, [2, 5, 9]]
        plot_body(ax, b3, 'o-', 'black')

        # Tie Rod
        b4 = HP[:, 8:10]
        plot_body(ax, b4, 'o-', 'cyan')

        # Spring/Damper Axis
        b5 = HP[:, 6:8]
        plot_body(ax, b5, 'o-', 'blue')

        # Wheel Axis
        b6 = HP[:, 12:14]
        plot_body(ax, b6, 'o-', 'grey')

        # Tyre Contact Patch
        ax.plot(HP[0, 14], HP[1, 14], HP[2, 14], '*', color='black')

        if virtual is True:
            plot_virtual(ax, HP)

    ax.set_aspect('equal', adjustable='box')
    plt.show()


def plot_virtual(ax, HP, color='silver'):

    VP = get_virtual(HP)

    ax.plot([0,0,0], VP[0, [0,2,4]], VP[1, [0,2,4]], '--', color=color)
    ax.plot([0,0,0], VP[0, [1, 3, 4]], VP[1, [1, 3, 4]], '--', color=color)
    ax.plot([0,0,0], VP[0, [4, 5, 6]], VP[1, [4, 5, 6]], '--o', color=color)


def get_virtual(HP):

    # Contact patch location
    CPy = HP[1,-1]
    CPz = HP[2,-1]

    # Interpolate inboard wishbone points to wheel center plane
    IUy = HP[1, 0] - HP[0, 0] * (HP[1, 0] - HP[1, 1]) / (HP[0, 0] - HP[0, 1])
    IUz = HP[2, 0] - HP[0, 0] * (HP[2, 0] - HP[2, 1]) / (HP[0, 0] - HP[0, 1])

    ILy = HP[1, 3] - HP[0, 3] * (HP[1, 3] - HP[1, 4]) / (HP[0, 3] - HP[0, 4])
    ILz = HP[2, 3] - HP[0, 3] * (HP[2, 3] - HP[2, 4]) / (HP[0, 3] - HP[0, 4])

    # Instant Center
    m1 = (HP[2, 2] - IUz) / (HP[1, 2] - IUy)  # slope of upper wishbone line
    m2 = (HP[2, 5] - ILz) / (HP[1, 5] - ILy)  # slope of lower wishbone line
    b1 = HP[2, 2] - m1 * HP[1, 2]  # intercept of upper wishbone line
    b2 = HP[2, 5] - m2 * HP[1, 5]  # intercept of lower wishbone line
    ICy = (b2 - b1) / (m1 + m2)  # instant center y coordinate
    ICz = m1 * ICy + b1  # instant center x coordinate

    # Roll Center
    RCy = 0  # assuming symmetric suspension geometry
    if math.isfinite(ICy):
        m = (CPz - ICz) / (CPy - ICy)  # slope of the n-line
        RCz = CPz - m * CPy  # roll center height (z coordinate)
    else:
        RCz = 0

    VP = np.array([[IUy, IUz],
                   [ILy, ILz],
                   HP[1:3, 2],
                   HP[1:3, 5],
                   [ICy, ICz],
                   [RCy, RCz],
                   [CPy, CPz]]).transpose()
    return VP
