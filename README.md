## CarND-PID-Project - Term 2

## Reflection 


### PID Controller

The PID controller is a control loop feedback method which consistenly corrects the cross-track-error (CTE) of the setpoint 
and the measured value. This correction is based on proprotional = P , integrational = I and differantial = D correction components. 
Therefore the name PID. The formular below presents PID controller algorithm:  

  	PID correction(t) = - Kp * CTE(t) - Ki * ∫ CTE(t) dt - Kd * d CTE(t)/dt 
 

#### P Component Kp * CTE(t)
 
The P component of the PID controller is proportional to the cross-track-error by the parameter Kp. 
It quickly corrects with the disadvantage of overshoots and oszilations. I.e. a high Kp parameter 
leads to faster oszilators versus a smaller one.
In terms of the vehicle simulator the P compenent quickly corrects error but with wobbling behavior therefore. 

#### I Component Ki * ∫ CTE(t) dt
 
The I component of the PID controller integrates the cross-track-error over time. It corrects
presistence errors and bias. The parameter Ki defines when the correction kicks in.
In terms of the vehicle simulator static errors were corrected by the I component.

#### D Component Kd * d CTE(t)/dt
 
The D component of the PID controller compensates the cross-track-error change rate. It can correct the overshooting
of the P compenent therefore. The parameter Kd defines how fast the overshoting is corrected. 
In terms of the vehicle simulators the D compenent can remove the wobbling behavior of the vehicle.

* Note: All components are sensitive to the speed of the vehicle. I.e. the PID values need to be smaller
        for high speeds versus lower speeds.


### PID Parameter tuning

I have started tuning with the PID parameters used in the lesson PID (0.2,0.004,3). Using these parameters the vehicle safely ran through the course. Then I manually optimized these parameters further.
Additionally I ran twiddle to calculate optimized values. I used 1800 steps for evaluating a parameter change. The steps number should be selected in the way that the run can be reproduced in terms of CTE scenorios. Too small step numbers may not result the same CTE scenarios for a parameter changes. The usage of initial PID parameter values (0, 0, 0) did work for twiddle algorithm here since the vehicle went off the road during optimization. Therefore I have used the PID parameter from the manual tuning (0.2,0.004,3). The parameter optimization process consumed a lot of timing running the simulator.
Therefore I switched back to manual parameter optimization in the end.
