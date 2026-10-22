package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;

public class IntakeIO {
    private DcMotor intakeMotor;

    public IntakeIO(HardwareMap hmap)
    {
        intakeMotor = hmap.get(DcMotor.class, intakeConstants.MOTOR_NAME);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(intakeConstants.MOTOR_REVERSED? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(intakeConstants.ZERO_POWER_BEHAVIOR);
    }
    public void SetPower(double voltage)
    {
        voltage = getVoltageCompensed(voltage);
        intakeMotor.setPower(voltage);
    }

    private double getVoltageCompensed(double voltage)
    {
        return voltage * VoltageCompensator.getVoltage() / intakeConstants.VOLTAGE_COMPENSATION;
    }
}
