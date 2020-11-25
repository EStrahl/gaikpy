# Introduction

gaikpy solves the inverse kinematics for every URDF based robot model using a genetic algorithm approach. No pretraining is needed. gaikpy has already integrated the NICO robot, but you can easily extend this with every robot with a URDF model at hand. Just take the URDF and define your chain ( containing the joints you want to use) , calculate the ik and maybe visualize it. 

If you use the library in an academic context, you can cite it by citing the paper below. We analysed the ik in a hybrid manipulation context for the NICO robot ( the measured calculation times in the paper are already outdated, gaikpy is much faster now ).



# History

# Installation

+ Clone the repository with git clone 
+ Change to the directory - "cd gaikpy"
+ Create a python environment - "python3 -m venv env"
+ Source your environment - "source ./env/bin/activate"
+ Install - "python setup.py develop" or "python setup.py install"
+ Run the test - "python setup.py test"
You should have no errors ( warnings are fine, the included libraries might throw some)

# Usage

Use the gaikpy to solve the full pose ik for you. 
See the example at ./examples/visualise_NICO.py . It will solve the ik of the NICO robot and visualise the results on the screen. Extend the example for your own needs.

# Integrate another robot

# License

GNU GENERAL PUBLIC LICENSE Version 3