import math
def roll_centre(HP):

    # Contact patch coordinates
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
    b1 = HP[2,2] - m1 * HP[1,2]  # intercept of upper wishbone line
    b2 = HP[2,5] - m2 * HP[1,5]  # intercept of lower wishbone line
    ICy = (b2 - b1) / (m1 + m2)  # instant center y coordinate
    ICz = m1 * ICy + b1  # instant center x coordinate

    # Roll Center
    RCy = 0  # assuming symmetric suspension geometry
    if math.isfinite(ICy):
        m = (CPz - ICz) / (CPy - ICy)  # slope of the n-line
        RCz = CPz - m * CPy  # roll center height (z coordinate)
    else:
        RCz = 0

    FVSAL = CPy - ICy

    return RCz, FVSAL
