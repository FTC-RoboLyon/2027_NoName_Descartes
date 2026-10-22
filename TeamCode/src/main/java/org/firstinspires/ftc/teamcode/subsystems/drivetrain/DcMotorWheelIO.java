package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.drive.WheelIO;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * FTC implementation of {@link WheelIO} using a {@code DcMotorEx} on a REV
 * Control/Expansion Hub.
 *
 * <p>Open-loop velocity now goes through a {@link FeedForwardModel} (kS + kV)
 * instead of a raw linear mapping: kS overcomes static friction, kV maps a wheel
 * speed (m/s) to a duty cycle. With kS = 0 and kV = 1 / MAX_WHEEL_SPEED_MPS this
 * reduces to the previous behaviour exactly.
 */
public class DcMotorWheelIO implements WheelIO {
    private final DcMotorEx motor;

    // Open-loop wheel feedforward: wheel speed (m/s) -> duty cycle [-1, 1].
    private final FeedForwardModel feedforward = new FeedForwardModel(
            DrivetrainConstants.WHEEL_KS,
            DrivetrainConstants.WHEEL_KV,
            DrivetrainConstants.WHEEL_KA);

    private double requestedVoltage;

    public DcMotorWheelIO(HardwareMap hardwareMap, String deviceName, boolean reversed) {
        motor = hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // zero the encoder at init
        motor.setMode(DrivetrainConstants.USE_CLOSED_LOOP
                ? DcMotor.RunMode.RUN_USING_ENCODER
                : DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Output is a duty cycle.
        feedforward.setOutputLimits(-1.0, 1.0);

        requestedVoltage = 0.0;
    }

    @Override
    public void updateInputs(WheelIOInputs inputs) {
        inputs.connected = true; // FTC has no per-motor "connected" flag for now
        inputs.positionMeters = motor.getCurrentPosition()
                / DrivetrainConstants.TICKS_PER_WHEEL_REV * DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS;
        inputs.velocityMetersPerSecond = motor.getVelocity()
                / DrivetrainConstants.TICKS_PER_WHEEL_REV * DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS;
        inputs.statorCurrentAmps = motor.getCurrent(CurrentUnit.AMPS);
        inputs.appliedVolts = requestedVoltage;
        inputs.temperatureCelsius = 0.0; // not exposed by the FTC SDK
    }

    @Override
    public void setVelocity(double metersPerSecond) {
        if (DrivetrainConstants.USE_CLOSED_LOOP) {
            double ticksPerSecond = metersPerSecond
                    / DrivetrainConstants.WHEEL_CIRCUMFERENCE_METERS
                    * DrivetrainConstants.TICKS_PER_WHEEL_REV;
            motor.setVelocity(ticksPerSecond);
            // The motor's internal PIDF compensates voltage; log a rough estimate.
            requestedVoltage = metersPerSecond / DrivetrainConstants.MAX_WHEEL_SPEED_MPS
                    * DrivetrainConstants.NOMINAL_VOLTAGE;
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
