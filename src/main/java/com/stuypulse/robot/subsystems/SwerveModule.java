package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.util.Motor;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An example swerve module where drive motor is controlled by a velocity 
 * controller and the turn motor is controlled by a profiled position controller
 * which is fed angles.
 * 
 * The velocity controllers includes a feedforward and a pid controller.
 * 
 * The position controller contains an angle pid controller and a feedforward
 * that will work on the derivative of the positional setpoints. There is also 
 * a filter on the setpoint that will apply a motion profile to them.
 */
public class SwerveModule extends SubsystemBase {

    /** the turn motor, target, and controller */
    private final Motor turnMotor;
    private final SmartAngle targetAngle;
    private final AngleController turnController;

    /** the drive motor, target, and controller */
    private final Motor driveMotor;
    private final SmartNumber targetVelocity;
    private final Controller driveController;

    public SwerveModule() {
        turnMotor = new Motor();
        targetAngle = new SmartAngle("Swerve Module/Target Angle", Angle.kZero).useDegrees();
        turnController = new AnglePIDController(0.1, 0.0, 0.0)
            .add(new Feedforward.Motor(0.1, 0.08, 0.0).angle())
            .setSetpointFilter(new AMotionProfile(1.0, 10.0));

        driveMotor = new Motor();
        targetVelocity = new SmartNumber("Swerve Module/Target Velocity", 0.0);
        driveController = new PIDController(0.1, 0.0, 0.0)
            .add(new Feedforward.Motor(0.1, 0.1, 0.1).velocity());
    }

    /** set the target state of the swerve module */
    public void setTargetState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, turnMotor.getAngle().getRotation2d());
        
        targetAngle.set(Angle.fromRotation2d(state.angle));
        targetVelocity.set(state.speedMetersPerSecond);
    }

    @Override
    public void periodic() {
        /** calculate output for turning motor */
        double turnOutput = turnController.update(targetAngle.get(), turnMotor.getAngle());
        turnMotor.set(turnOutput);

        /** calculate output for drive motor */
        double driveOutput = driveController.update(targetVelocity.get(), driveMotor.getVelocity());
        driveMotor.set(driveOutput);

        /** put measurements on the network */
        SmartDashboard.putNumber("Swerve Module/Measured Velocity", driveController.getMeasurement());
        SmartDashboard.putNumber("Swerve Module/Measured Angle", turnController.getMeasurement().toDegrees());
        // NOTE: it is best to use getMeasurement() if any filters are applied
    }

}