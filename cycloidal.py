#!/usr/bin/python
'''
Mike Dawson-Haggerty
1/30/2014

Inspired by:
http://www.zincland.com/hypocycloid

Implemented from:
'Gear geometry of cycloid drives', Chen BingKui et al.
http://download.springer.com/static/pdf/96/art%253A10.1007%252Fs11431-008-0055-3.pdf
Equations 10, 11
'''

import numpy as np

import trimesh


def cycloidal_profile(count_pin,
                      count_cam,
                      eccentricity,
                      radius_pin,
                      radius_pattern,
                      resolution=16):
    '''
    Return a (n,2) curve representing a cyloidal gear profile.

    Parameters
    -------------
    count_pin:      int, number of pins in the radial pattern
    count_cam:      int, number of lobes on the disc.
                          for a regular disc, equal to count_pin-1
    eccentricity:   float, magnitude of eccentricity
    radius_pin:     float, radius of individual pin
    radius_pattern: float, radius of pin centers
    resolution:     float, number of points per degree

    Returns
    ------------
    profile: (n,2) float, ordered points on curve in 2D space
    '''
    Rz = radius_pattern  # radius of pin pattern
    rz = radius_pin      # radius of pin
    e = eccentricity    # eccentricity
    Zb = count_pin       # number of pins
    Zg = count_cam       # tooth count on gear

    Ze = Zb / (Zb - Zg)
    Zd = Zg / (Zb - Zg)
    K1 = (e * Zb) / (Rz * (Zb - Zg))

    # in the paper they say you should calculate this numerically...
    psi = np.linspace(0, np.pi * 2, int(resolution * 360))

    denom_B = np.sqrt(1 + K1**2 - 2 * K1 * np.cos(Zd * psi))
    cos_B = np.sign(Zb - Zg) * ((K1 * np.sin(Ze * psi)) -
                                np.sin(psi)) / denom_B
    sin_B = np.sign(Zb - Zg) * ((-K1 * np.cos(Ze * psi)) +
                                np.cos(psi)) / denom_B

    x = Rz * np.sin(psi) - e * np.sin(Ze * psi) + rz * cos_B
    y = Rz * np.cos(psi) - e * np.cos(Ze * psi) - rz * sin_B

    profile = np.column_stack((x, y))

    return profile


def dual_cycloidal(eccentricity=.07,
                   count_pin=24,
                   radius_pin=.125,
                   radius_pattern=2.25,
                   input_count=3,
                   input_radius=.5625,
                   input_pattern=1.3125):
    '''
    Generate the profiles, pins, and holes for a regular dual- disc
    cycloidal drive. This design has two discs operation 180 degrees
    out of phase to minimize vibration.

    Parameters
    ------------
    eccentricity:   float, magnitude of eccentricity
    count_pin: int, number of fixed pins
    radius_pin:     float, radius of a fixed pin
    radius_pattern: float, radius of pin pattern
    input_count:    int, number of holes in the cycloidal disc
    input_radius:   float, radius of the holes in the cycloidal disc
    input_pattern:  float, radius of the hole pattern in the disc

    Returns
    -------------
    drive: Path2D object, with two disc layers and a pin layer
    '''

    # half a tooth spacing for transforming disc to pin pattern
    spacing = .5 * np.pi / (count_pin - 1)

    # get a disc profile, with a very dense sampling
    a = trimesh.load_path(cycloidal_profile(count_pin=count_pin,
                                            count_cam=count_pin - 1,
                                            eccentricity=eccentricity,
                                            radius_pin=radius_pin,
                                            radius_pattern=radius_pattern,
                                            resolution=32))

    # replace the polyline entity with a bajillion points with a
    # tightly fit B-Spline
    a.simplify_spline(smooth=1e-6)

    # the second disc has the same profile, with a different transform
    b = a.copy()
    # for the first disc, apply the transform to line up with the pins
    a.apply_transform(trimesh.transformations.planar_matrix(
        offset=[-eccentricity, 0.0],
        theta=spacing))
    a.apply_layer('cam_disc_A')
    # do the same for the second disc
    b.apply_transform(trimesh.transformations.planar_matrix(
        offset=[eccentricity, 0.0],
        theta=-spacing))
    b.apply_layer('cam_disc_B')

    # generate the fixed pins
    pins = trimesh.path.creation.circle_pattern(pattern_radius=radius_pattern,
                                                circle_radius=radius_pin,
                                                count=count_pin)
    pins.apply_layer('pins')

    # add the holes for the
    holes_A = trimesh.path.creation.circle_pattern(pattern_radius=input_pattern,
                                                   circle_radius=input_radius,
                                                   center=[-eccentricity, 0.0],
                                                   count=input_count)
    holes_A.apply_layer('cam_disc_A')

    holes_B = trimesh.path.creation.circle_pattern(pattern_radius=input_pattern,
                                                   circle_radius=input_radius,
                                                   center=[eccentricity, 0.0],
                                                   count=input_count)
    holes_B.apply_layer('cam_disc_B')

    # concatenate all of the paths into a single drawing
    drive = a + b + pins + holes_A + holes_B

    return drive


if __name__ == '__main__':
    drive = dual_cycloidal()
    drive.export('cycloidal_drive.dxf')

    drive.show()
