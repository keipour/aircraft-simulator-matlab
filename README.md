# Multirotor Modeling and Control Tutorial

This repository is a simulator for introducing the basic modeling and control concepts of multirotors. It allows users to interactively define new multirotor architectures, develop and tune controllers, and analyze the developed architectures and controllers. The software is for educational purposes, but it is written in a way that is easily expandable and can be used for research as well.

This tutorial is provided as part of the Air Lab Tutorial Week 2020 at Carnegie Mellon University. Please visit [here](https://theairlab.org/summer2020/) for more information about this Virtual School.

## Dependencies
The simulator is written and tested in MATLAB R2019b with all the Toolboxes provided by Carnegie Mellon University. The tutorials are Live Scripts. 

There are no dependencies for the simulator and no MATLAB toolbox has been used. However, the code is not tested without the toolboxes, so please inform us if there is an unnoticed dependency.

The code will probably work with other recent releases of MATLAB, but it is not tested with versions older than R2019b. Please let us know if you have a successful run or if you face issues on older versions so we can update this README.

## Instructions
Please clone this repo. The code should work right out of the box. There are three tutorial files:

* `task1.mlx`: Introduces the *simulator*, sets up the *multirotor model* and the *controller*, creates the *attitude controller* and asks the user to tune the PID attitude controller.

* `task2.mlx`: Develops a *position controller* and asks the user to write a PID loop and to calculate the desired attitude based on the position and yaw inputs.

* `task3.mlx`: Develops a new *fully-actuated multirotor architecture*, and develops a controller for this new architecture.

## Contact
Azarakhsh Keipour (keipout [at] cmu.edu)

Please contact us for comments, questions, issues and suggestions. For more tutorials and information about the Air Lab, please visit [our lab's website](https://theairlab.org/)

## License
[This software is BSD licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2020, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
