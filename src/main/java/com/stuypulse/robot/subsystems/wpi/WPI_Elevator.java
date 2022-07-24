package com.stuypulse.robot.subsystems.wpi;

import com.stuypulse.robot.util.Motor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// if not using TrapezoidProfileSubsystem
public class WPI_Elevator extends SubsystemBase {
    
    // motor controlling the elevator
    private final Motor motor;

    // internal timer
    private final Timer timer;

    // for calculating setpoints
    private final Constraints constraints;
    private State setpoint;
    private State goal;

    private double lastVelocity;

    // for calculating outputs
    private final PIDController feedback;
    private final ElevatorFeedforward feedforward;

    public WPI_Elevator() {
        motor = new Motor();
        timer = new Timer();

        constraints = new Constraints(3, 2);
        goal = new State(0.0, 0.0);
        setpoint = new State(0.0, 0.0); // assumed to start at the bottom

        lastVelocity = 0.0;

        feedback = new PIDController(0.1, 0.0, 0.0);
        feedforward = new ElevatorFeedforward(1, 0.1, 0.1, 0.1);
    }

    public void setHeight(double height) {
        this.goal = new State(height, 0);
    }

    @Override
    public void periodic() {
        double period = timer.get();
        timer.reset();

        var profile = new TrapezoidProfile(constraints, goal, setpoint);
        var state = profile.calculate(period);
        setpoint = state;

        double fb = feedback.calculate(motor.getPosition(), setpoint.position);
        double ff = feedforward.calculate(setpoint.velocity, (setpoint.velocity - lastVelocity) / period);
        lastVelocity = setpoint.velocity;

        motor.set(fb + ff);
    }
}
