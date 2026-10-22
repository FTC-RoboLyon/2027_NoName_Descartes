package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

public final class intakeConstants {
    public final static ControlMode MainControlMode = ControlMode.DUTY_CYCLE;
    public final static ControlMode EmergencyControlMode = ControlMode.MANUAL_DUTY_CYCLE;

    //--- Intake Motor ---
    public final static String MOTOR_NAME = "intakeMotor";
    public final static boolean MOTOR_REVERSED = true;
    public final static DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
    public final static double VOLTAGE_COMPENSATION = 10.0;

    //--- Voltage ---
    public final static double REST = 0.0;
    public final static double INTAKE_POWER = 1.0;
}
