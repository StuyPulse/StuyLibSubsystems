package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.util.Motor;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.angle.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    private final Motor motor;
    private final SmartAngle setpoint;
    private final AngleController controller;

    public Arm() {
        controller = new AnglePIDController(0.1, 0.0, 0.0)
            .add(new ArmFeedforward(1.0, 1.0, 0.1, 0.1).angle())
            .setSetpointFilter(new AMotionProfile(3.0, 3.0));
     
        motor = new Motor();
        setpoint = new SmartAngle("Arm/Target Angle", Angle.kZero).useDegrees();
    }

    public void setAngle(Angle angle) {
        setpoint.set(angle);
    }

    @Override
    public void periodic() {
        double output = controller.update(setpoint.getAngle(), motor.getAngle());
        motor.set(output);

        SmartDashboard.putNumber("Arm/Measurement Angle", controller.getMeasurement().toDegrees());
    }

}
