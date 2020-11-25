#!/usr/bin/env python
# -*- coding: utf-8 -*-

from gaikpy import chain
from gaikpy import robot
from gaikpy import chain_definitions
import time
import pickle

nico_path = "./resources/urdf/nico/complete_modified_bounds.urdf"

rightchain = chain.Chain.from_urdf_file(nico_path , base_elements=["torso:11", "r_shoulder_z", "right_shoulder:11", 
                                        "r_shoulder_y", "right_collarbone:11", "r_arm_x", "right_upper_arm:11", "r_elbow_y", 
                                        "right_lower_arm:11", "r_wrist_z","right_wrist:11", "r_wrist_x", 
                                        "right_palm:11", "r_ringfingers_x"],
                                        active_joints_mask=[False, True, True, True, True, True, True, False, False, False])

leftchain = chain.Chain.from_urdf_file(nico_path , base_elements=["torso:11", "l_shoulder_z", "left_shoulder:11",
                                    "l_shoulder_y", "left_collarbone:11", "l_arm_x", "left_upper_arm:11", "l_elbow_y", 
                                    "left_lower_arm:11", "l_wrist_z","left_wrist:11", "l_wrist_x", 
                                    "left_palm:11", "l_ringfingers_x"],
                                       active_joints_mask=[False, True, True, True, True, True, True, False, False, False])

#Print out the link chains for the two arms
print("--- Active Link names ----")
print("--- Right: " + str(rightchain.get_all_active_joint_names()) )
print("--- Left: " + str(leftchain.get_all_active_joint_names()) )
#input()

#create the robot model for visualisation
nico_right_chain=chain_definitions.nico_right_chain_active
nico=robot.robot(nico_path,nico_right_chain)

#get forward kinematics of the robot and print first link
fk = nico.r_model.link_fk()
print(fk[nico.r_model.links[1]])

with open('./resources/data/nico/nico_right_20_new.p', 'rb') as f:
            sample_set = pickle.load(f)

#input()

for sample in sample_set:

    (joi,sfwr)=sample

    nico.update_target(sfwr)

    #nico.update_robot_pose(joi)

    joi = rightchain.active_to_full(joi)
    print ("Original : \n" + str(joi))

    #animate(joi)

    #joi[2:] = 0
    #fwr=self.rightchain.forward_kinematics(joi,full_kinematics=False)

    #rightchain.plot(joi, ax,  target=sfwr, show=plot)


    #rightchain.plot(joi, ax, show=True )
    #plt.draw()
    #plt.pause(0.05)


    '''
    joi_nr=2

    fkp = robot.link_fk(cfg=cfg)
    print ("urdfpy : \n" + str(fkp[robot.links[joi_nr]]))

    fwr=rightchain.forward_kinematics(joi,full_kinematics=True)
    print ("ikpy : \n" + str(fwr[joi_nr]))
    '''

    #input()
    #time.sleep(1)

    ik_kor = rightchain.inverse_kinematics(sfwr, method="ga_simple",include_orientation=True,
                        numGenerations=1000,max_iter=100000,dist_acc=0.01,or_acc=0.1,
                        multiproc=False,orientation_weight=-1)
    
    #ik_kor = rightchain.inverse_kinematics(sfwr, method="ga_simple",include_orientation=False,
    #                    max_iter=100000,dist_acc=0.01,
    #                    multiproc=False,orientation_weight=-1)
    #from IPython.core.debugger import set_trace
    #set_trace()
    #joi = rightchain.active_to_full(ik_kor, [0] * len(rightchain.links))
    joi=rightchain.active_from_full(ik_kor)
    print ("ga_ik: \n" + str(ik_kor))
    nico.update_robot_pose(joi)
    #input()
    time.sleep(1)