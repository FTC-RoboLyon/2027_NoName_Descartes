package org.firstinspires.ftc.teamcode.subsystems.feeder;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.teamcode.subsystems.feeder.FeederConstants;


public class FeederIO {
    private DcMotor feederMotor;

    

    public FeederIO(HardwareMap hmap)
    {
        feederMotor = hmap.get(DcMotor.class, FeederConstants.MOTOR_NAME);

        feederMotor.setDirection(FeederConstants.MOTOR_REVERSED ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        feederMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setZeroPowerBehavior(FeederConstants.MOTOR_ZERO_POWER_BEHAVIOR);
    }

    public void setMotorPower(double power)
    {
        feederMotor.setPower(getVoltageCompensed(power));
    }

    private double getVoltageCompensed(double power)
    {
        power = power * FeederConstants.NOMINAL_VOLTAGE / VoltageCompensator.getVoltage();
        if(Math.abs(power) > 1.0)
        {
            power /= Math.abs(power);
        }
        return power;
    }
}
