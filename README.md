# Multirotor Modeling and Control Tutorial

This repository is a simulator for introducing the basic modeling and control concepts of multirotors. It allows users to interactively define new multirotor architectures, develop and tune controllers, and analyze the developed architectures and controllers. The software is for educational purposes, but it is written in a way that is easily expandable and can be used for research as well.

## Dependencies
The simulator is written and tested in MATLAB R2019b with all the Toolboxes provided by Carnegie Mellon University. The tutorials are Live Scripts. 

There are no dependencies for the simulator and no MATLAB toolbox has been used. However, the code is not tested without the toolboxes, so please inform us if there is an unnoticed dependency.

The code will probably work with other recent releases of MATLAB, but it is not tested with versions older than R2019b. Please let us know if you have a successful run or if you face issues on older versions so we can update this README.

## Instructions
Please clone this repo. The code should work right out of the box. There are three tutorial files:

* `task1.mlx`: Introduces the *simulator*, sets up the *multirotor model* and the *controller*, creates the *attitude controller* and asks the user to tune the PID attitude controller.

* `task2.mlx`: Develops a *position controller* and asks the user to write a PID loop and to calculate the desired attitude based on the position and yaw inputs.

* `task3.mlx`: Develops a new *fully-actuated multirotor architecture*, and develops a controller for this new architecture.

## Copyright
Copyright (c) 2020 Carnegie Mellon University

## Author
Azarakhsh Keipour (keipout [at] cmu.edu)

Please don't hesitate to contact us for any comments or issues.