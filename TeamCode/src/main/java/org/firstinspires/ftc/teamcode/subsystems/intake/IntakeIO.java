package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;

public class IntakeIO {
    private DcMotor frontIntakeMotor;
    private DcMotor backIntakeMotor;

    public IntakeIO(HardwareMap hmap)
    {
        frontIntakeMotor = hmap.get(DcMotor.class, IntakeConstants.FRONT_MOTOR_NAME);
        backIntakeMotor = hmap.get(DcMotor.class, IntakeConstants.BACK_MOTOR_NAME);

        frontIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntakeMotor.setDirection(IntakeConstants.FRONT_MOTOR_REVERSED? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        frontIntakeMotor.setZeroPowerBehavior(IntakeConstants.INTAKE_ZERO_POWER_BEHAVIOR);

        backIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backIntakeMotor.setDirection(IntakeConstants.BACK_MOTOR_REVERSED? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        backIntakeMotor.setZeroPowerBehavior(IntakeConstants.INTAKE_ZERO_POWER_BEHAVIOR);
    }
    public void SetPower(double frontIntakePower, double backIntakePower)
    {
        frontIntakeMotor.setPower(getVoltageCompensed(frontIntakePower));
        backIntakeMotor.setPower(getVoltageCompensed(backIntakePower));
    }

    private double getVoltageCompensed(double power)
    {
        double output = power * IntakeConstants.VOLTAGE_COMPENSATION / VoltageCompensator.getVoltage();
        if (Math.abs(output) > 1.0)
        {
            output /= Math.abs(output);
        }
        return output;
    }
}
