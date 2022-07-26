This is a simulation project for wheel-leg bikebot crossing rugged terrain using Raisim
==========================
#### Introduction
Bicycle robot has high-speed and high-energy efficiency on off-road terrains, but is unable to keep balance at low speed and run on uneven terrain.
So we design a wheel-leg hybrid bikebot control system to provide assistive torque to balance the bikebot at low speed or off-road terrain conditions.

A model predictive control is designed for the leg assistive
actuation to take advantage of the leg-ground interaction force
and balance torque.

The following video shows the created simulation scenario in RaiSim.

[![Watch the video](http://img.youtube.com/vi/YZMGbSeeLnE/0.jpg)](https://youtu.be/YZMGbSeeLnE)

#### Control system
The following figure is schematic of control system.

<img src="https://user-images.githubusercontent.com/35949664/180992503-6add7e8f-449c-43a9-b3db-c213f8392e6b.png" width="500" /><br/>

At first build the whole system dynamics model and get the state space equation.

We further use the mpc form to calculate the proper leg-ground interaction force and use qpOASES solver to make the computation faster.

#### Simulation

First generate the 3D model of bikebot by solidworks and then use algorithm to control the robot in Raisim.

<img src="https://user-images.githubusercontent.com/35949664/180995867-0e63222a-d377-466a-8fa0-fbfe32b1369e.png" width="500" /><br/> 

Finally we create rugged terrain with the height of the topography varies as a function of the xy coordinate as ℎ=|0.07sin(7𝑥)−0.07cos(4𝑦+2)| m.

<img src="https://user-images.githubusercontent.com/35949664/177945929-40849e89-40bb-4c1d-a30c-74fcec280bb1.png" width="500" /><br/>
