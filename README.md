# A Fully Actuated Quadrotor UAV with a Propeller Tilting Mechanism: Modeling and Control
Matlab/Simulink implementation of a Dynamic feedback Linearization scheme for a 6 DoF UAV. This is an implementation of the paper by Marcin Odelga, Paolo Stegagno and Heinrich H. Bülthoff [A Fully Actuated Quadrotor UAV with a Propeller Tilting Mechanism: Modeling and Control](https://ieeexplore.ieee.org/document/7576784).

This repo contains
- `iskew.m`: function that given a skew symmetrix matrix generated from a vector, it returns said vector. It is the operator $\[ \cdot \]_{\vee}$ in the paper;
- `matrix_A.m`: let $\dot{\phi}$ be the vector of Roll-Pitch-Yaw derivatives and $\omega$ the vector of angular velocities, this function returns the matrix $A(\phi)$ such that $\dot{\phi}=A(\phi)\omega$. It takes $\phi$ as input;  
- `rpy_rotation.m`,`euler_rotation.m`: functions that given an axis sequence and a sequence of angles, return the rotation matrix in the RPY or Euler convention;  
- `elem_rot_mat.m`: given an axis 'x', 'y', or 'z' and an angle, it returns the associated rotation matrix $\in SO(3)$  
- `dynamic_model.m`: script that contains the UAV dynamic terms, along with $\dot{\textbf{c}},\textbf{J}^{\ast}$ used in the controller  This file also contains the dynamic parameters of the UAV and generates the function ```quadrotor_dynamics.m``` used in the simulink simulation;  
- `quadrotor_controller.m` : function that implements the control law for the quadrotor;  
- `quadrotor_dynamics.m`: auto-generated function from the dynamics script;  
- `quadrotor_model.slx` simulink block scheme. Inside it is possible to find various simulations that can be tried;  
- `plotting.m` : script that plots the data after running a simulation. It is also possible to save the figures if desired.

## Block scheme overview
![image](https://user-images.githubusercontent.com/72447693/232256160-c73f32e9-dd0a-4d61-ac2d-e1013039ecb6.png)

Note the algebraic loop due to the fact that the inputs $\omega_m$ depend on the acceleration of the quadrotor algebraically, and these acceleration are given in feedback.
