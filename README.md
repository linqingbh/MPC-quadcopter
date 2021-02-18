# MPC Quadcopter
Model Predictive Control for an autonomous quadcopter (UAV)

* System modelling
	* State-Space model
	* Simplification and linearization
* Controller design
	* PID SISO control approach
	* MPC MIMO control approach
	* Observers: Kalman filter
* Simulation
	* PID simulation
	* MPC simulation (linear model)
	* Real-Time simulation (non-linear model)


## Model predictive control (MPC)
### Algorithm	
![](img/mpc.png)

* Initialization of the variables
* Get the augmented incremental model and the parameters of the control trajectories vector (DeltaU) based on the state-space system, the control horizon (Nc) and prediction horizon (Np) 
  * Augmented incremental model
  
	![](img/states_vector.png)
	
	![](img/ss_model.png)
	
	![](img/om.png)

  * Incremental control trajectory vector (DeltaU) obtained from the cost function minimization
  
  	![](img/DeltaU.png)	
	
	![](img/Rs.png) 
	
	![](img/R.png)
	
	![](img/F_G.png)
	
* Calculate the constant part of DeltaU

* Control loop

	* Read input signal 
	* Calculate the variable part of DeltaU
	* Add both parts and apply the receding horizon control extracting first element
	* Get the control signal
	
		![](img/uk.png)
		
	* Get the incremental system state
	
		![](img/states_vector.png)	


## MPC with output constraints

* In the initialization variables: 

	* Create M matrix
	
* In the control loop: 

	* Calculate the gamma matrix with the constraints of the output	
	* Recalculate DeltaU from optimization with restrictions
	
	![](img/constraints.png)


## References
Sistemas de control en tiempo real para aplicaciones industriales: Teoría, problemas y prácticas - Ramón Guzmán 2020
Dynamic Modeling and Control of a Quadrotor Using Linear and Nonlinear Approaches - H. M. Nabil ElKholy - 2014
Decentralized Navigation of Multiple Quad-rotors using Model Predictive Control - I. Khan - 2017
