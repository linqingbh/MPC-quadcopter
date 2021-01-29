# Model Predictive Control
Real-Time Model Predictive Control: MPC, MPC with constraints, DMC and GPC

![](https://upload.wikimedia.org/wikipedia/commons/thumb/1/11/MPC_scheme_basic.svg/1280px-MPC_scheme_basic.svg.png)

## MPC
### Implementation algorithm

* Initialization of the variables
* Get the augmented incremental model and the parameters of the control trajectories vector (DeltaU) based on the state-space system, the control horizon (Nc) and prediction horizon (Np) 
  * Augmented incremental model



  * Incremental control trajectory vector (DeltaU)  
* Calculate the constant part of DeltaU

* Control loop

	* Read input signal 
	* Calculate the variable part of DeltaU
	* Add both parts and apply the receding horizon control extracting first element
	* Get the control signal
	* Get the incremental system state
