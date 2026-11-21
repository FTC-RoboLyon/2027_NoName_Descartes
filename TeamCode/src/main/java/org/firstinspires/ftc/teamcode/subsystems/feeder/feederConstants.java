package org.firstinspires.ftc.teamcode.subsystems.feeder;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

public class FeederConstants {
    public final static ControlMode MainControlMode = ControlMode.DUTY_CYCLE;
    public final static ControlMode EmergencyControlMode = ControlMode.MANUAL_DUTY_CYCLE;

    // --- Motor ---
    public final static String MOTOR_NAME = "feederMotor";
    public final static boolean MOTOR_REVERSED = false;
    public final static ZeroPowerBehavior MOTOR_ZERO_POWER_BEHAVIOR = ZeroPowerBehavior.FLOAT;
    public final static double NOMINAL_VOLTAGE = 12.0;

    // --- Voltage ---
    public final static double REST = 0.0;
    public final static double FEEDING_POWER = 1.0;
}
