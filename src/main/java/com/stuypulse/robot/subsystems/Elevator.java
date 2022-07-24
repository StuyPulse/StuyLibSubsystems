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
* the "profile". If the profile is sampled repeatedly, the set of positions generated will bring 
* the system to its target position while abiding by a given maximum velocity and acceleration.
* 
* This fact is the thinking behind the controllers for this elevator:
* 
*  - There is a positional controller which will simply compare the current height to the
*    height provided by the motion profile.
* 
*  - To take advantage of the velocity between the profiled setpoints, we can have a 
*    velocity feedforward do a lot of the work to counteract the delay of the PID loop.
* 
* 
*/
public class Elevator extends SubsystemBase {

    /** The controller, which will encapsulate position and velocity controllers given instructions by a motion profile. */
    private final Controller controller;

    /** The setpoint that will control the position of the motor / height of the elevator */
    private SmartNumber setpoint;

    /** The motor being controlled by a target height. */
    private final Motor motor;

    public Elevator() {
        controller = new PIDController(0.1, 0.1, 0.1)
            .add(new Feedforward.Elevator(1.0, 6.9, 4.0, 2.0).position())
            .setSetpointFilter(new MotionProfile(1, 20.0));

        motor = new Motor();
        setpoint = new SmartNumber("Elevator/Setpoint (meters)", 0.0);
    }

    /** set the target height of the elevator */
    public void setHeight(double height) {
        setpoint.set(height);
    }

    @Override
    public void periodic() {
        /** calculate output for the elevator motor */
        double output = controller.update(setpoint.doubleValue(), motor.getPosition());
        motor.set(output);

        SmartDashboard.putNumber("Elevator/Measurement (meters)", controller.getMeasurement());
    }
}