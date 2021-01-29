# Model Predictive Control
Real-Time Model Predictive Control simulations: MPC, MPC with constraints, DMC and GPC


![](img/mpc.png)


## MPC
### Algorithm

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
		
### Example: Position control of a DC motor

![](img/DCmotor_MPC.jpg)

## MPC with constraints (output)

* In the initialization variables: Create M matrix
* In the control loop: 
	* Calculate the gamma matrix with the constraints of the output
	* Recalculate DeltaU from optimization with restrictions
	
	![](img/constraints.png)
