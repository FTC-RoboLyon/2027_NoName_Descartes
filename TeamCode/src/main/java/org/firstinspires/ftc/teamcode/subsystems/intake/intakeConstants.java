package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

public final class intakeConstants {
    public final static ControlMode MainControlMode = ControlMode.DUTY_CYCLE;
    public final static ControlMode EmergencyControlMode = ControlMode.MANUAL_DUTY_CYCLE;

    // --- Front Intake Motor ---
    public final static String FRONT_MOTOR_NAME = "isFrontIntakeMotor";
    public final static boolean FRONT_MOTOR_REVERSED = true;

    // --- Back Intake Motor ---
    public final static String BACK_MOTOR_NAME = "backIntakeMotor";
    public final static boolean BACK_MOTOR_REVERSED = true;

    // --- Both Motors ---
    public final static DcMotor.ZeroPowerBehavior INTAKE_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
    public final static double VOLTAGE_COMPENSATION = 10.0;

    // --- Voltage ---
    public final static double REST = 0.0;
    public final static double INTAKE_POWER = 1.0;
}
