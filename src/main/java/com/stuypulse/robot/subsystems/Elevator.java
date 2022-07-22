package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.util.Motor;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.*;
import com.stuypulse.stuylib.control.feedforward.*;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
* An example motion profiled Elevator subsystem. 
* 
* A motion profile takes in the target position and generates a new intermediate position along 
* the "profile".If the profile is sampled repeatedly, the set of positions generated will bring 
* the system to its target position while abiding by a given maximum velocity and acceleration.
* 
* This fact is the thinking behind the controllers for this elevator:
* 
*  - There is a positional controller which will simply compare the current height to the
*    height provided by the motion profile.
* 
*  - To take advantage of the maximum velocity between the profile setpoints, we can have an 
*    additional velocity controller built-in.motion profiled int
* 
*    This will take the derivative of the profile setpoints to get velocity. It can then act 
*    as a standard velocity controller that uses feedforward and PID to calculate an output.
*
*/
public class Elevator extends SubsystemBase {

    /** The controller, which will encapsulate position and velocity controllers given instructions by a motion profile. */
    private final Controller controller;

    /** The setpoint that will control the position of the motor / height of the elevator */
    private Number setpoint;

    /** The motor being controlled by a target height. */
    private final Motor motor;

    public Elevator() {
        // Full Controller:
        // controller = new PIDController(0.1, 0.1, 0.1)
        //     .and(
        //             new PIDController(0.1, 0.0, 0.0)
        //                 .and(new Feedforward.Elevator(1.0, 1.0, 0.1, 0.0).velocity())
        //                 .setSetpointFilter(new Derivative())
        //                 .setMeasurementFilter(new Derivative())
        //     )
        //     .setSetpointFilter(new JerkLimit(1, 20.0));

        /** The part of the controller that controls velocity */
        Controller velocity = new PIDController(0.1, 0.0, 0.0)
            .and(new Feedforward.Elevator(1.0, 1.0, 0.1, 0.0).velocity())
            .setSetpointFilter(new Derivative())
            .setMeasurementFilter(new Derivative());

        /** The part of the controller that controls position */
        Controller position = new PIDController(0.1, 0.1, 0.1);

        /** The final controller combines velocity and position control with a 
            * trapezoidal profile on the position setpoints AND feeding the derivative
            * of the positon setpoints to the velocity controllers.
            */
        controller = position
            .and(velocity)
            .setSetpointFilter(new JerkLimit(1, 20.0));

        motor = new Motor();

        setpoint = new SmartNumber("Elevator/Setpoint (meters)", 0.0);
    }

    @Override
    public void periodic() {
        double output = controller.update(setpoint.doubleValue(), motor.getPosition());
        motor.set(output);

        SmartDashboard.putNumber("Elevator/Measurement (meters)", controller.getMeasurement());
    }
}