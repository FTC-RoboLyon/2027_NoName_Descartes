package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.drive.WheelIO;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;

/**
 * FTC implementation of {@link WheelIO} using a {@code CRServo} on a REV
 * Control/Expansion Hub (or a DCMotor connected to a SparkMini controller).
 *
 * <p>Open-loop velocity now goes through a {@link FeedForwardModel} (kS + kV)
 * instead of a raw linear mapping: kS overcomes static friction, kV maps a wheel
 * speed (m/s) to a duty cycle. With kS = 0 and kV = 1 / MAX_WHEEL_SPEED_MPS this
 * reduces to the previous behaviour exactly.
 */
public class CRServoWheelIO implements WheelIO {
    private final CRServo motor;

    // Open-loop wheel feedforward: wheel speed (m/s) -> duty cycle [-1, 1].
    private final FeedForwardModel feedforward = new FeedForwardModel(
            DrivetrainConstants.WHEEL_KS,
            DrivetrainConstants.WHEEL_KV,
            DrivetrainConstants.WHEEL_KA);

    private double requestedVoltage;

    public CRServoWheelIO(HardwareMap hardwareMap, String deviceName, boolean reversed) {
        motor = hardwareMap.get(CRServo.class, deviceName);
        motor.setDirection(reversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);

        // Output is a duty cycle.
        feedforward.setOutputLimits(-1.0, 1.0);

        requestedVoltage = 0.0;
    }

    @Override
    public void updateInputs(WheelIOInputs inputs) {
        inputs.connected = true; // FTC has no per-motor "connected" flag for now
        inputs.positionMeters = 0.0; //SparkMini controller doesn't support motor encoders
        inputs.velocityMetersPerSecond = 0.0; //SparkMini controller doesn't support motor encoders
        inputs.statorCurrentAmps = 0.0; //SparkMini controller can't measure the current intensity
        inputs.appliedVolts = requestedVoltage;
        inputs.temperatureCelsius = 0.0; // not exposed by the FTC SDK
        
    }

    @Override
    public void setVelocity(double metersPerSecond) {
        if (DrivetrainConstants.USE_CLOSED_LOOP) {
            throw new IllegalStateException("Guy calm down, remember your drivetrain motors are CRServos");
        } else {
            // kS + kV feedforward, clamped to [-1, 1] by the model's output limits.
            double duty = feedforward.calculate(metersPerSecond);
            motor.setPower(duty);
            requestedVoltage = duty * DrivetrainConstants.NOMINAL_VOLTAGE;
        }
    }

    @Override
    public void setVoltage(double volts) {
        // FTC has no direct voltage control; approximate via duty cycle.
        requestedVoltage = UtilsRBL.clamp(-DrivetrainConstants.NOMINAL_VOLTAGE,
                volts,
                DrivetrainConstants.NOMINAL_VOLTAGE);
        motor.setPower(requestedVoltage / DrivetrainConstants.NOMINAL_VOLTAGE);
    }
}
