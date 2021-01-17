# Control-of-Autonomous-quadrotor
toward control of quadrotors



*******************************************************************************************************************
*******************************************************************************************************************
***************         						             ******************************
***************              CONTROL OF AUTONOMOUS QUADROTOR                         ******************************
***************           							     ******************************
*******************************************************************************************************************
*******************************************************************************************************************


 		################# Those programs are done by Oussama Bouaiss #################


*******************************************************************************************************************
*******************************************************************************************************************

You have all permission to use and develop the programs.
Doors are open for any suggested improvement.you are welcome

*******************************************************************************************************************
*******************************************************************************************************************


The file contains:

* Angles calculation.m
* PID simulink file
* IBS simulink file
* MPC simulink file
* drawing.m

Those programs are aimed to be a tutorial for researches and student to understand the art of quadrotor control.
Those design are not perfect as this was not the aim, focus was about creating a research tuturial so many 
possible modification will be effective for better performace.
 
the model was developed from Newton Euler formalism, then the three method control was based on nested loop design.

Each method is well explained in the paper and derivation of controller is shown.

Noise & disturbance were added by considering white Gaussian noises


*******************************************************************************************************************
*******************************************************************************************************************
- For better performance you can change the controllers parameters
     => Change PIDs parameters on PID sim.
     => Change the IBS parameters in IBS control block
     => Manipulate the horizon time, control inputs horizon, ... in MPC sim.
 
- In case of slow simulation by fsolve interpreted function, use the direct approximation method which are way quicker.



*******************************************************************************************************************
*******************************************************************************************************************



For PID & IBS sim, a direct running in enough, results of simulation can be seen directly in scops or use Drawing.m 
to draw/plot them from the work space 
For MPC, you need to run the system first without any controller, to attribut the equilibrium point, then open MPC c
controller and call for a design, add its parameters as desired, run it then.



*******************************************************************************************************************
*******************************************************************************************************************

In case of changing the desired trajectory, be sure that it is physically physible by the model first to not fall 
in the space of irrealizable trajectories.
In this case you will be treating path following problem instead and time referenc comparaison is not valid.

*******************************************************************************************************************
*******************************************************************************************************************


- Contact me in case of any problem. 

*******************************************************************************************************************
*******************************************************************************************************************



last update 01/15/2020




####################################################################################################################
© 2021 GitHub, Inc.
