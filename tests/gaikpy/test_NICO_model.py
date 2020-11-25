import sys

import unittest
import numpy as np
import params
from gaikpy import chain
from gaikpy import robot
from gaikpy import chain_definitions

import pickle
import math3d as m3d
import math
import random
import time

import gaikpy

import matplotlib.pyplot as plt
from gaikpy import utils


visualisation=True

nico_path = "./resources/urdf/nico/complete_modified_bounds.urdf"

nico_right_chain=chain_definitions.nico_right_chain_active

nicorightchain = chain.Chain.from_urdf_file(nico_path , base_elements=["torso:11", "r_shoulder_z", "right_shoulder:11",
                                "r_shoulder_y", "right_collarbone:11", "r_arm_x", "right_upper_arm:11", "r_elbow_y", "right_lower_arm:11", "r_wrist_z",
                                "right_wrist:11", "r_wrist_x", "right_palm:11", "r_ringfingers_x"],
                                        active_joints_mask=[False, True, True, True, True, True, True, False, False, False])


if visualisation:
    nico=robot.robot(nico_path,nico_right_chain,off_screen=False,ego_view=True,visualisation=True)



class TestNICORobot(unittest.TestCase):
    def setUp(self):
        self.rightchain = chain.Chain.from_urdf_file(nico_path , base_elements=["torso:11", "r_shoulder_z", "right_shoulder:11",
                                                                                                                                                  "r_shoulder_y", "right_collarbone:11", "r_arm_x", "right_upper_arm:11", "r_elbow_y", "right_lower_arm:11", "r_wrist_z",
                                                                                                                                                  "right_wrist:11", "r_wrist_x", "right_palm:11", "r_ringfingers_x"],
                                        active_joints_mask=[False, True, True, True, True, True, True, False, False, False])

        self.leftchain = chain.Chain.from_urdf_file(nico_path , base_elements=["torso:11", "l_shoulder_z", "left_shoulder:11",
                                                                                                                                                 "l_shoulder_y", "left_collarbone:11", "l_arm_x", "left_upper_arm:11", "l_elbow_y", "left_lower_arm:11", "l_wrist_z",
                                                                                                                                                 "left_wrist:11", "l_wrist_x", "left_palm:11", "l_ringfingers_x"],
                                       active_joints_mask=[False, True, True, True, True, True, True, False, False, False])


        
        with open('./resources/data/nico/nico_right_20_new.p', 'rb') as f:
            self.sample_set = pickle.load(f)
        #print("Chains:")
        #print(rightchain)
        #print("All Links right: " + str(rightchain.get_all_link_names()))
        #print("All active links right: " + str(rightchain.get_all_active_link_names()))
        #print(leftchain)

    def tearDown(self):
        pass

    @classmethod
    def tearDownClass(cls):
        #Close the virtualisation windows
        if visualisation:
                nico.viewer.close_external()
                while nico.viewer.is_active:
                    pass

    def test_forward_ki(self):
        
        for sample in self.sample_set:
            (joi,sfwr)=sample
            joi = self.rightchain.active_to_full(joi)
            fwr=self.rightchain.forward_kinematics(joi,full_kinematics=False)
            self.assertEqual(len(sfwr.tolist()),len(fwr.tolist()))
            np.testing.assert_almost_equal(sfwr, fwr, decimal=7, err_msg='fki not equal', verbose=True)
            #self.assertListEqual(sfwr.tolist(),fwr.tolist())
    
    def _test_a_max_move(self):

        max_dist=0
        max_ang=0
        step = math.pi*2.0/4096.0
        import copy

        #Get usable initial
        for sample in self.sample_set:
            (sjoi,sfwr)=sample

            sjoi = self.rightchain.active_to_full(sjoi)
            fwr=self.rightchain.forward_kinematics(sjoi,full_kinematics=False)
            
            for t in range (100000):
                indi=random.randrange(0,len(sjoi))
                old_sjoi=copy.deepcopy(sjoi)
                sjoi[indi]=sjoi[indi]+step
                new_sjoi=sjoi  
                sjoi = self.rightchain.active_to_full(sjoi)  
                fwr_new=self.rightchain.forward_kinematics(sjoi,full_kinematics=False)
                euclidean_distance  = math.sqrt(np.linalg.norm(fwr[:3, -1] - fwr_new[:3, -1]))
                angle_distance=utils.angle_diff(fwr,fwr_new)
                if angle_distance > math.pi:
                    angle_distance=2*math.pi-angle_distance
                #print ("Eu: " + str(euclidean_distance))
                fwr=copy.deepcopy(fwr_new)
                if euclidean_distance > max_dist:
                    print ("Eu: " + str(euclidean_distance))
                    max_dist=euclidean_distance
                    print ("Max: " + str(max_dist) + " \n  joints: " + str(new_sjoi) +  "\nex-joints: " + str(old_sjoi))
                if angle_distance > max_ang:
                    print ("Ang: " + str(angle_distance))
                    max_ang=angle_distance
                    print ("Max: " + str(max_ang) + " \n  joints: " + str(new_sjoi) +  "\nex-joints: " + str(old_sjoi))

        input()
    


    def inverse_ki(self,method,orientation=False,acc_ex=.001,acc_or_ex=.1):
        
        if orientation:
                or_str=" with or "
        else:
                or_str=" without or "

        print ("---- Test: " + method +" " + or_str + " ---- " )
        for sample in self.sample_set:
            (sjoi,sfwr)=sample
            
            if visualisation:
                nico.update_target(sfwr)

            target = sfwr[:3, 3]
            if method!="ga_simple":
                joi = self.rightchain.inverse_kinematics(sfwr, method= method,  \
                    include_orientation=orientation,max_iter=1000000,orientation_weight=0.5)
            else:
                if orientation:
                    joi = self.rightchain.inverse_kinematics(sfwr, method= method,  \
                        include_orientation=orientation,numGenerations=100,max_iter=100000,dist_acc=0.001,or_acc=0.045,
                        multiproc=True,orientation_weight=-1)         

                else:
                    joi = self.rightchain.inverse_kinematics(sfwr, method= method,  \
                        include_orientation=orientation,numGenerations=100,max_iter=100000,or_acc=None,dist_acc=0.001,
                        multiproc=False)         

            #joi = self.rightchain.active_to_full(joi)
            
            fwr=self.rightchain.forward_kinematics(joi,full_kinematics=False)
            
            if visualisation:
                #fjoi = joi
                fjoi = nicorightchain.active_from_full(joi)
                nico.update_robot_pose(fjoi)
                #time.sleep(2)
            
            #input()

            euclidean_distance  = math.sqrt(np.linalg.norm(fwr[:3, -1] - target))

            
            print ("Euklidean dist "+method+": " + str(euclidean_distance))
            
            self.assertEqual(len(sfwr.tolist()),len(fwr.tolist()))
            self.assertTrue(euclidean_distance<acc_ex, method + " Euklidean distace " + or_str + " is " + str(euclidean_distance))
            if (orientation):
                angle_distance=gaikpy.utils.angle_diff_tait(fwr,sfwr)
                print ("Angle dist "+method+": " + str(angle_distance))
                self.assertTrue(angle_distance<acc_or_ex, method + " Angle distace is " + str(angle_distance))
    
    def inverse_ki_pos_only(self,method,acc_ex=.001):
        self.inverse_ki(method,orientation=False,acc_ex=acc_ex)

    def inverse_ki_with_or(self,method,acc_ex=.001,acc_or_ex=.1):
        self.inverse_ki(method,orientation=True,acc_ex=acc_ex,acc_or_ex=acc_or_ex)

    def _test_inverse_ki_de(self):
        
        #self.inverse_ki_pos_only("de")
        #self.inverse_ki_with_or("de",acc_or_ex=0.2)
        pass

    def test_inverse_ki_BFGS(self):
        
        #pass
        self.inverse_ki_pos_only("L-BFGS-B",acc_ex=.2)
        

    def test_inverse_ki_SLSQP(self):
        
        #pass
        self.inverse_ki_pos_only("SLSQP",acc_ex=.005)
    
    def test_inverse_ki_ga(self):
        
        self.inverse_ki_pos_only("ga_simple")
        self.inverse_ki_with_or("ga_simple")

    

if __name__ == '__main__':
    unittest.main(verbosity=2, argv=[sys.argv[0]])
