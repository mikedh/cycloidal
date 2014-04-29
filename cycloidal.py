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
import matplotlib.pyplot as plt
import sdxf

def cycloidal_profile(count_pin, 
                      count_cam,
                      eccentricity,
                      radius_pin,
                      radius_pattern):
  
    Rz = radius_pattern  # radius of pin pattern
    rz = radius_pin      # radius of pin
    e  = eccentricity    # eccentricity
    Zb = count_pin       # number of pins
    Zg = count_cam       # tooth count on gear
    
    Ze = Zb / (Zb-Zg)
    Zd = Zg / (Zb-Zg)
    
    K1 = (e*Zb) / (Rz*(Zb-Zg))
    
    # in the paper they say you should calculate this numerically...
    psi_max = np.pi*2
    psi     = np.linspace(0, psi_max, 8*360)
    
    denom_B = np.sqrt(1 + K1**2 - 2*K1*np.cos(Zd * psi))
    cos_B   = np.sign(Zb-Zg) *  ((K1 * np.sin(Ze*psi)) - np.sin(psi)) / denom_B

    sin_B   = np.sign(Zb-Zg) * ((-K1 * np.cos(Ze*psi)) + np.cos(psi)) / denom_B
    x = Rz*np.sin(psi) - e*np.sin(Ze*psi) + rz*cos_B
    y = Rz*np.cos(psi) - e*np.cos(Ze*psi) - rz*sin_B

    return np.column_stack((x,y))

def transform_points(points, xy_theta):
    xy    = xy_theta[0:2]
    theta = xy_theta[2]
    R = np.eye(3)
    c = np.cos(theta)
    s = np.sin(theta)
    R[0:2, 0:2] = [[c, -s], [s, c]]
    R[0:2, 2]   = xy
    transformed = np.dot(R, 
                         np.column_stack((points, 
                                          np.ones(len(points)))).T).T[:,0:2]
    return transformed
 
class Display:
    def __init__(self, plot=False, filename=None):
        self.plot     = plot
        self.save     = filename <> None
        self.filename = filename
        if self.save: self.dxf = sdxf.Drawing()
        
    def circles(self,
                radius_circle,
                radius_pattern = 0.0,
                count          = 1,
                offset         = [0,0,0],
                layer          = 'circles'):
                
        sample      = np.linspace(0, np.pi*2, 90)
        circle      = np.column_stack((np.cos(sample), np.sin(sample))) * radius_circle
        angle       = np.arange(offset[2], offset[2]+np.pi*2, (np.pi*2/(count)))
        pin_centers = np.column_stack((np.cos(angle), np.sin(angle))) * radius_pattern + offset[0:2]
        for center in pin_centers:
            if self.plot: 
                plt.plot(*(circle + center).T, color='k')
            if self.save: 
                self.dxf.append(sdxf.Circle(center = center, 
                                            radius = radius_circle, 
                                            layer  = layer))
    def profile(self,
                profile, 
                offset = [0,0,0],
                layer = 'profile'):
        profile = transform_points(profile, offset)
        if self.plot: 
            plt.plot(*profile.T)
        if self.save:
            self.dxf.append(sdxf.PolyLine(points = profile.tolist(),
                                          layer  = layer))
    def close(self):
        if self.save:
            self.dxf.saveas(self.filename)
        if self.plot: 
            plt.axes().set_aspect('equal', 'datalim')  
            plt.show()

def pack_radius(pin_radius, pin_count):
    n = float(pin_count)
    R = pin_radius / (np.sin(np.pi / n))
    return R
    
def generate_rolling():
    pin_radius     = .5/2
    pin_count      = 17
    eccentricity   = 0.055


    pattern_radius = pack_radius(pin_radius, pin_count)

    inner_cam = cycloidal_profile(pin_count, 
                                  pin_count-1,
                                  eccentricity,
                                  pin_radius,
                                  pattern_radius)
    inner_cam += [eccentricity,0]
    
    outer_cam  = cycloidal_profile(pin_count, 
                                   pin_count+1,
                                   eccentricity,
                                   pin_radius,
                                   pattern_radius) 
    outer_cam  += [-eccentricity,0]
      
    display = Display(plot=True, filename='rolling_cycloidal.dxf')
    
    display.circles(pin_radius,
                    pattern_radius,
                    pin_count)
    display.profile(inner_cam, 
                    layer = 'inner_cam')
    display.profile(outer_cam, 
                    layer = 'outer_cam')
    display.close()
    
def generate_regular():
    pin_radius     = 0.25 / 2.0
    eccentricity   = 0.070
    pin_count      = 20
    pattern_radius = 1.75

    display = Display(plot=True, filename='cycloidal.dxf')
    
    inner_cam = cycloidal_profile(pin_count, 
                                  pin_count-1,
                                  eccentricity,
                                  pin_radius,
                                  pattern_radius)
    # draw the pin pattern
    display.circles(pin_radius, pattern_radius, pin_count, layer='pins')
    
    # output hole pattern
    display.circles((19/25.4)*.5, .75, 3, offset=[eccentricity, 0,0], layer='outcam_A')
    display.circles((19/25.4)*.5, .75, 3, offset=[-eccentricity, 0,0], layer='outcam_B')

    display.profile(inner_cam, 
                    offset = [eccentricity,0, .5*np.pi/(pin_count-1)], 
                    layer  = 'outcam_A')
    display.profile(inner_cam, 
                    offset = [-eccentricity,0, -.5*np.pi/(pin_count-1)], 
                    layer  = 'outcam_B')
    
    display.close()

if __name__ == '__main__': 
    generate_regular()       
    generate_rolling()