# Introduction

![Nico using ik](https://github.com/knowledgetechnologyuhh/gaikpy/blob/main/nico_ik.jpg?raw=true "NICO ik")

gaikpy is a pure python approach to solve the inverse kinematics for every URDF modelled robot.

gaikpy solves the inverse kinematics for every URDF based robot model using a genetic algorithm approach. No pretraining is needed. gaikpy is completely realised in python. gaikpy has already integrated the [NICO robot](https://www.inf.uni-hamburg.de/en/inst/ab/wtm/research/neurobotics/nico.html), but you can easily extend this with every robot with a URDF model at hand. Just take the URDF and define your chain ( containing the joints you want to use) , calculate the ik and maybe visualize it. 

If you use the library in an academic context, you can cite it by citing the paper below. We analysed the ik in a hybrid manipulation context for the NICO robot ( the measured calculation times in the paper are already outdated, gaikpy is much faster now ).

Neuro-Genetic Visuomotor Architecture for Robotic Grasping   
Matthias Kerzel, Josua Spisak, Erik Strahl, Stefan Wermter
Artificial Neural Networks and Machine Learning – ICANN 2020, pages 533-545 - 2020.

https://www2.informatik.uni-hamburg.de/wtm/publications/2020/KSSW20/ICANN_2020__Neuro_Genetic_Visuomotor_Framework_Preprint.pdf

# History and credits

The Neuro Inspired COmpanion ( or [NICO](https://www.inf.uni-hamburg.de/en/inst/ab/wtm/research/neurobotics/nico.html) ) is the robot we developed at the [Knowledge Technology group](https://www.inf.uni-hamburg.de/en/inst/ab/wtm/about.html) at the [University of Hamburg](https://www.uni-hamburg.de/en.html) to fullfill our [research needs](https://www2.informatik.uni-hamburg.de/wtm/publications/2017/KSMNHW17/NICO_RO-MAN_2017-PREPRINT-2017.pdf). 

Our philosophy is to have a full python based interface for the NICO ( besides our ROS based interface ), so we searched first for an existing python based ik library. We found [ikpy](https://github.com/Phylliade/ikpy), which is a great software, but was at least at time we started not able to solve the full-pose ik ( but only the position ) for our NICO robot. 

As we favour the idea of bioinspired algoritms in our group and one of our former students has [developed a genetic based ik approach in C#](https://ieeexplore.ieee.org/document/7866587), we took parts of this concept and adapted this to our python approach, which you see here with gaikpy.


![Nico using ik](https://github.com/knowledgetechnologyuhh/gaikpy/blob/main/nico_opt.gif?raw=true "NICO ik")

# Installation

## From the pypi repository

+ create a python environment

    python3 -m venv env
+ activate the environement

    source ./env/bin/activate
+ get gaikpy and the depening packages

    pip install gaikpy
+ run the demo

    demo.py

## Using the souce code from github

+ Clone the repository with git clone

    git clone https://github.com/knowledgetechnologyuhh/gaikpy.git
+ Change to the directory

    cd gaikpy
+ Create a python environment

    python3 -m venv env
+ Activate your environment 

    source ./env/bin/activate
+ Install

    python setup.py develop 
or 

    python setup.py install
+ Run the test

    python setup.py test
You should have no errors ( warnings are fine, the included libraries might throw some)
+ Run the demo

    python demo.py


# Build the docs

+ cd gaikpy/docs
+ make html
+ browse at gaikpy/docs/build/html

# Usage

Use the gaikpy to solve the full pose ik for you. 

The pip package and the github rep contain both the NICO urdf model, so that you use it instantly.

The demo.py program uses the NICO model and calculates the ik for some test values.

Take a look at demo.py and adapt it to you usecase or robot:

Start with your imports

    #!/usr/bin/env python
    # -*- coding: utf-8 -*-

    import gaikpy
    from gaikpy import chain
    from gaikpy import robot
    from gaikpy import chain_definitions
    import time
    import pickle
    import os


We need a URDF, which is the ROS, but as well in other context, model representation of a robot
Here we take take NICO model that is included in the package

    nico_path = gaikpy.get_nico_urdf_path()+"/complete_modified_bounds.urdf"

We have to define a chain containing the URDF and a list of joint / link combinations with all joints 
taking part in he movement. Below is a definiion o the right NICO arm

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


For visualization we built he a full robot model
again using the URDF

    nico_right_chain=chain_definitions.nico_right_chain_active
    nico=robot.robot(nico_path,nico_right_chain)

We use some poses, where we can be sure, that they are mechanically 
reachable (cause we have used forward kinematics to generate the dataset, see this in the examples about generating movement examples)

    with open(gaikpy.get_nico_data_path()+'/nico_right_20_new.p', 'rb') as f:
                sample_set = pickle.load(f)

Go through all the samples in file
This contains pairs of poses in Eukledian and in the joint space 
Of cours we use only the Eukledian ones in this example

    for sample in sample_set:

        # joi is the data in joint space for the robot (not used)
        # sfwr is the pose in Euklidan space
        (joi,sfwr)=sample
        print ("Original pose: \n" + str(sfwr))

        # Show a target pointer in the scene, so that we know, which pose NICOs hand shoul take 
        nico.update_target(sfwr)

Calculate the IK, get the robots joint values

        ik_kor = rightchain.inverse_kinematics(sfwr, method="ga_simple",include_orientation=True,
                            numGenerations=1000,max_iter=100000,dist_acc=0.01,or_acc=0.1,
                            multiproc=True,orientation_weight=-1)
        
        # Change the joint data to the full joint representation
        joi=rightchain.active_from_full(ik_kor)

        # Get the Eukledian pose using orward kinematics
        fwr=rightchain.forward_kinematics(ik_kor,full_kinematics=False)

        #Let us compare the results
        print ("gaikpy result: \n" + str(fwr))
        
Update the pose on the visualised NICO

        nico.update_robot_pose(joi)
        
        #Wait a time to display the result
        time.sleep(0.3)


# Use other robots

Using other robots is very easy, if you have a URDF model of the robot at hand. Just take the URDF model and list the joints and links of your active chain until the end effector. Just adapt the demo.py or the NICO example ./examples/visualise_NICO.py to see how it works.
We will integrate other robots by default in the future. 

# License

GNU GENERAL PUBLIC LICENSE Version 3