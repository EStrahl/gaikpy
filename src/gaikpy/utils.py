#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Utility functions for gaikpy to calculate especially errors, fitness and accuracy 
"""

def angle_diff_tait(fwk,target_frame):
    """ Returns the rotation difference between two poses in Euler space 

    Parameters
    ----------
    fwk : numpy matrix
        pose
    target_frame : 
        target pose
    """

    import transforms3d
    import math

    # k=transforms3d.euler.EulerFuncs()
    i_al, i_be, i_ga = transforms3d.taitbryan.mat2euler(fwk[:3, :3])
    t_al, t_be, t_ga = transforms3d.taitbryan.mat2euler(target_frame[:3, :3])
    d_al = i_al - t_al
    d_be = i_be - t_be
    d_ga = i_ga - t_ga
    orientation_distance = math.sqrt(d_al * d_al + d_be * d_be + d_ga * d_ga)
    return(orientation_distance)

def angle_diff(fwk,target_frame):
    """position difference in Euler space using quaternions 

    Parameters
    ----------
    fwk : numpy matrix
        pose
    target_frame : 
        target pose
    

    Returns
    -------
    float 
        difference in rad
    """

    import math3d as m3d
    import numpy as np

    target_quad = m3d.quaternion.UnitQuaternion(m3d.orientation.Orientation(target_frame[:3, :3]))
    recent_quad = m3d.quaternion.UnitQuaternion(m3d.orientation.Orientation(fwk[:3, :3]))
    target_vector = target_quad.get_vector_part()
    recent_vector = recent_quad.get_vector_part()

    return np.arccos(np.clip(np.dot(target_vector, recent_vector), -1.0, 1.0))


def quad_diff(fwk,target_frame):
    """position difference in quaternion space 

    Parameters
    ----------
    fwk : numpy matrix
        pose
    target_frame : 
        target pose
    

    Returns
    -------
    float 
        difference in quad
    """

    import math3d as m3d

    targetQuad = m3d.quaternion.UnitQuaternion(m3d.orientation.Orientation(target_frame[:3, :3]))
    recentQuad = m3d.quaternion.UnitQuaternion(m3d.orientation.Orientation(fwk[:3, :3]))
    return(targetQuad.ang_dist(recentQuad))