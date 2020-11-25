Introduction

History

Installation

Clone the repository with git clone 
change to the directory - "cd gaikpy"
create a python environment - "python3 -m venv env"
source your environment - "source ./env/bin/activate"
install - "python setup.py develop" or "python setup.py install"
run the test - "python setup.py test"
You should have no errors ( warnings are fine, the included libraries might throw some)

Usage

Use the gaikpy to solve the full pose ik for you. 
See the example at ./examples/visualise_NICO.py . It will solve the ik of the NICO robot and visualise the results on the screen. Extend the example for your own needs.

Integrate your robot