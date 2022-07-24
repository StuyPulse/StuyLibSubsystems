package com.stuypulse.robot.subsystems.wpi;

import com.stuypulse.robot.util.Motor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

// using trapezoid profile subsystem
public class WPI_Elevator2 extends TrapezoidProfileSubsystem {

    // motor controlling elevator
    private final Motor motor;

    // calculate acceleration
    private double lastVelocity;

    // calculate outputs
    private final PIDController feedback;
    private final ElevatorFeedforward feedforward;

    public WPI_Elevator2() {
        super(new Constraints(3, 2));

        motor = new Motor();

        lastVelocity = 0.0;

        feedback = new PIDController(1.0, 0.0, 0.0);
        feedforward = new ElevatorFeedforward(1.0, 1.0, 1.0, 0.1);
    }

    @Override
    protected void useState(State state) {
        double ff = feedforward.calculate(state.velocity, (state.velocity - lastVelocity) / 0.02);
        lastVelocity = state.velocity;

        double fb = feedback.calculate(state.position, motor.getPosition());

        motor.set(ff + fb);
    }
}
