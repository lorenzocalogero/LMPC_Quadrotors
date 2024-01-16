# Learning Model Predictive Control for Quadrotors Minimum-Time Flight in Autonomous Racing Scenarios

## Description ##

This repository collects the code implementing a Learning Model Predictive Control (LMPC) algorithm for quadrotors autonomous racing.

The proposed algorithm allows to define a highly customizable 3D race track, in which multiple types of obstacles can be inserted. The controller is then able to autonomously find the best trajectory minimizing the quadrotor lap time, by learning from data coming from previous flights within the track, ensuring also the avoidance of all the obstacles therein.

Moreover, novel relaxation approaches for the LMPC optimization problem are presented, allowing to reduce it from a mixed-integer nonlinear program to a quadratic program.

<img src="https://user-images.githubusercontent.com/49368313/229520729-51367004-4f7f-4f1b-846b-c8f6be21c8aa.jpg" alt="drawing" height="260"/> <img src="https://user-images.githubusercontent.com/49368313/229524421-51d4480b-08b3-4c42-9b1d-058e288b6b2b.jpg" alt="drawing" height="260"/> <img src="https://user-images.githubusercontent.com/49368313/229520768-decec024-fc3e-4343-a6d6-e1b03d34dac6.jpg" alt="drawing" height="260"/>

## Prerequisites ##

For running the code, it is required to install the third-party toolbox "YALMIP", available [here](https://yalmip.github.io/ "YALMIP").

## References ##

The code is based on the following work:

* L. Calogero, M. Mammarella, and F. Dabbene, "Learning Model Predictive Control for Quadrotors Minimum-Time Flight in Autonomous Racing Scenarios," _IFAC-PapersOnLine_, vol. 56, no. 2, pp. 1063–1068, 2023 [[PDF](https://www.researchgate.net/publication/375850281)]
