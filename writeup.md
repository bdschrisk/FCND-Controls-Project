## Project: Building a Controller
![Estimation](/images/overview.png)

---

## Rubric Points
### Here I will consider the [rubric](https://review.udacity.com/#!/rubrics/1643/view) points individually and describe how I addressed each point in my implementation.

---

### Explain the Implementation

#### 1. Implemented body rate control in C++.
To implement the Body-Rate Controleler the commanded moment was calculated using a proportional difference of the desired body rate and the current estimated body rate.  The P controller provides a proportional gain to account for any loss in the system.

Finally the moments of inertia for each angle are applied to each individual body rate before being inputted to the thrust controller.

The final P gain values for the Body Rate Controller were tuned to:
```
Kp(PQR) = 46.0, 54.0, 10.0
```

#### 2. Implement roll pitch control in C++.
To implement the Roll-Pitch Controller the rotation matrix coming from the body frame (attitude) of the vehicle is used in conjunction with the mass moments for each direction of acceleration.

The collective thrust is used to balance the vehicle at a stationary point in space using the mass, which provides the baseline of direction.
```
T_c = -thrust / mass
```

Then the desired accelerations are accounted for with the baseline.
```
Bc = X_acc / T_c
```

Since the banking of the vehicle is determined by the rotation force, we apply a proportional difference to the rotaton rate and desired direction.
```
B_dot = Kp(Bank) * (Bc - Ba)
```

Finally the commanded pitch and roll rate is determined by the difference in the corresponding local acceleration in the roll and pitch direction.
```
PQR_c = R(1, 0:1) * B_dot(0) - R(0, 0:1) * B_dot(1)
PQR_c = PQR_c / b_z 
```
*Where b_z is the normalised angle in all directions*

The P gain value for the Roll Pitch Controller was tuned to:
```
Kp(Bank) = 8.8
```

#### 3. Implement altitude controller in C++.
The Altitude controller is responsible for returning the overall thrust to maintain a specific altitude.

First, the differences in desired acceleration and velocities are taken.
```
z_err = (posZCmd - posZ);
z_err_dot = (velZCmd - velZ);
```

Next, the proportional position and velocity parameters are computed to adjust for errors in the model's down acceleration (NED frame).
```
p_term = Kp(PosZ) * z_err;
d_term = Kp(VelZ) * z_err_dot;
```
Then, the integral gain:
```
integratedAltitudeError += (z_err * dt);
i_term = Ki(PosZ) * integratedAltitudeError;
```

The input to the altitude control loop is the computed P+I value:
```
u_1_bar = p_term + d_term + i_term + z_dot_dot;
```
*Where z_dot_dot is the desired acceleration in the down direction.*

Lastly, the thrust is calculated by taking the difference in the acceleration of Gravity, the relative angle of the vehicle and then accounting for the Newtons of the vehicle.
```
thrust = (u_1_bar - this->GRAVITY) / b_z;
thrust = mass * -thrust;
```
*Where b_z is the normalised angle in all directions*

This negative collective thrust is then fed into the Roll-Pitch Controller.  The negation of the thrust is required to produce upwards force by working against gravity.

The P gain values for the Altitude Controller were tuned to:
```
Kp(PosZ) = 34.0
Ki(PosZ) = 34.0
Kp(VelZ) = 9.0
```

#### 4. Implement lateral position control in C++.
The Lateral Position Controller is responsible for computing horizontal accelerations (X+Y)and modelling deficiencies introduced in the coordination system using a P controller.

First the difference in desired and estimated positition and velocities are taken:
```
z_dot = (posCmd - pos);
z_dot_dot = (velCmd - vel);
```

Lastly, the result is the combined control values:
``` 
accelCmd = Kp(PosXY) * z_dot + Kp(VelXY) * z_dot_dot + accelCmdFF;
```
*Where accelCmdFF is the feed-forward acceleration from a previous timestep*

The final commanded acceleration is then fed to the Body Rate Controller.

The P gain values for the Lateral Position Controller were tuned to:
```
Kp(PosXY) = 27.0
Kp(VelXY) = 12.0
```

#### 5. Implement yaw control in C++.
The Yaw Controller provides as input to the Body Rate Controller a resolved yaw value coming from the estimator and the desired yaw rate, which looks as follows:
```
yaw_cmd = Kp(Yaw) * N(yawCmd - yaw)
```
*Where `N()` is an angle normalisation function to be between -pi and +pi*

The P gain value for the Yaw Controller was tuned to:
```
Kp(Yaw) = 3.6
```

#### 6. Implement calculating the motor commands given commanded thrust and moments in C++.
Finally, the amount of thrust provided to each motor is dependent upon each of the different sub-controllers and their respective outputs.

To start, the X, Y, and Z directional moments about each axis are scaled by the mechanical aspects of the vehicle and the torque about the Z axis.
```
l = L * 0.5 * sqrt(2.0F);
F_x = momentCmd.x / l;
F_y = momentCmd.y / l;
F_z = momentCmd.z / kappa;
```
*Where L is the arm length of the vehicle - i.e. the center to motor distance.  Kappa is the amount of rotational torque produced around the Z axis per Nm of thrust produced by each motor.*

To calculate the thrust for each motor, we add the entire forces for each motor in their respective direction.  In the right-hand coordinate system, forward is Y, thus the Y force (F_y) is positive, while reverse is negative.  Left (F_x) is then positive, and negative for the right movement. The rotation (F_z) is then the equalised force between each of the opposing motors, which are spinning in opposite directions to counteract torque.
```
FL_motor = 0.25 * (f + F_x + F_y - F_z)
FR_motor = 0.25 * (f - F_x + F_y + F_z)
RL_motor = 0.25 * (f + F_x - F_y + F_z)
RR_motor = 0.25 * (f - F_x - F_y - F_z)
```
*Where 0.25 is the scaling factor for each motor, which is simply the whole thrust (1) divided by the number of motors (4)*