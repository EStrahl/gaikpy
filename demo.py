#!/usr/bin/env python
# -*- coding: utf-8 -*-

import gaikpy
from gaikpy import chain
from gaikpy import robot
from gaikpy import chain_definitions
import time
import pickle
import os

# We need a URDF, which is he ROS, but as well in other context, model representation of a robot
# Here we take take NICO model that is included in the package
nico_path = gaikpy.get_nico_urdf_path()+"/complete_modified_bounds.urdf"

# We have to define a chain containing the URDF and a list of joint / link combinations wit all joints 
# taking part in he movement. Below is a definiion o the right NICO arm
rightchain = chain.Chain.from_urdf_file(
            # The path to he URDF
            nico_path , 
            # The chain: We start with a link and the first joint up to the last joint involved
            base_elements=["torso:11", "r_shoulder_z", "right_shoulder:11", 
            "r_shoulder_y", "right_collarbone:11", "r_arm_x", "right_upper_arm:11", "r_elbow_y", 
            "right_lower_arm:11", "r_wrist_z","right_wrist:11", "r_wrist_x", 
            "right_palm:11", "r_ringfingers_x"],
            # Not all the joints listed have to take part in the movement
            # This is a mask blending out joints inbetween the chain 
            # (here we do not want to involve the finger joints) 
            active_joints_mask=[False, True, True, True, True, True, True, False, False, False])


# For visualization we built he a full robot model
# again using the URDF
nico_right_chain=chain_definitions.nico_right_chain_active
nico=robot.robot(nico_path,nico_right_chain)

# We use some poses, where we can be sure, that they are mechanically 
# reachable (cause we have used forward kinematics to generate the dataset, see this in the examples)
with open(gaikpy.get_nico_data_path()+'/nico_right_20_new.p', 'rb') as f:
            sample_set = pickle.load(f)

# Go through all the samples in file
# The contains pairs of poses in Eukledian and in the joint space 
# Of cours we use only the Eukledian ones
for sample in sample_set:

    # joi is the data in joint space for the robot (not used)
    # sfwr is the pose in Euklidan space
    (joi,sfwr)=sample
    print ("Original pose: \n" + str(sfwr))

    # Show a target pointer in the scene, so that we know, which pose NICOs hand shoul take 
    nico.update_target(sfwr)

    # Calculate the IK, get the robots joint values
    ik_kor = rightchain.inverse_kinematics(sfwr, method="ga_simple",include_orientation=True,
                        numGenerations=1000,max_iter=100000,dist_acc=0.01,or_acc=0.1,
                        multiproc=True,orientation_weight=-1)
    
    # Change the joint data to the full joint representation
    joi=rightchain.active_from_full(ik_kor)

    # Get the Eukledian pose using orward kinematics
    fwr=rightchain.forward_kinematics(ik_kor,full_kinematics=False)

    #Let us compare the results
    print ("gaikpy result: \n" + str(fwr))
    
    # And update the pose on the visualised NICO
    nico.update_robot_pose(joi)
    
    #Wait a time to display the result
    time.sleep(0.3)