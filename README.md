# Model Predictive Control
Real-Time Model Predictive Control simulations: MPC, MPC with constraints, DMC and GPC


![](img/mpc.jpg)


## MPC
### Algorithm

* Initialization of the variables
* Get the augmented incremental model and the parameters of the control trajectories vector (DeltaU) based on the state-space system, the control horizon (Nc) and prediction horizon (Np) 
  * Augmented incremental model
  
	![](img/states_vector.jpg)
	
	![](img/ss_model.jpg)
	
	![](img/om.jpg)

  * Incremental control trajectory vector (DeltaU)
  
  	![](img/DeltaU.jpg)
	
	![](img/Rs.jpg) 
	
	![](img/R.jpg)
	
	![](img/F_G.jpg)
	
* Calculate the constant part of DeltaU

* Control loop

	* Read input signal 
	* Calculate the variable part of DeltaU
	* Add both parts and apply the receding horizon control extracting first element
	* Get the control signal
	
		![](img/uk.jpg)
		
	* Get the incremental system state
	
		![](img/states_vector.jpg)
		
### Example: Position control of a DC motor

![](img/DCmotor_MPC.jpg)

## MPC with constraints
