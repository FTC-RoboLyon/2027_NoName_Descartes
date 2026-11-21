package org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

@Config
public class FlywheelConstants {

    public static final ControlMode MainConntrolMode = ControlMode.VELOCITY_DUTYCYCLE_PID_FF;
    public static final ControlMode EmergencyConntrolMode = ControlMode.MANUAL_VELOCITY;
    public static final String FLYWHEEL_MOTOR_NAME = "flywheel";
    public static double FLYWHEEL_NOMINAL_VOLTAGE = 12.4;

    public static final double ENCODEUR_MOTOR_RATIO = 1.0;
    public static final double FLYWHEEL_TICKS_PER_ROTATION = 8192.0;

    public static double FLYWHEEL_MOTOR_KP = 0.0030; //TUNEME
    public static double FLYWHEEL_MOTOR_KI = 0.0007; //TUNEME
    public static double FLYWHEEL_MOTOR_KD = 0.00075; //TUNEME

    public static double FLYWHEEL_MOTOR_KV = 0.0004434589800443459; //TUNEME
    public static double FLYWHEEL_MOTOR_KS = 0.335; //TUNEME

    public static double FLYWHEEL_VELOCITY_FILTER_ALPHA = 0.1; //TUNEME

    public static double FLYWHEEL_TOLERANCE = 50; //TUNEME

    public static double FLYWHEEL_SPEED_NEAR = 1000; //TUNEME IN RPM
    public static double FLYWHEEL_SPEED_FACED_WITH_CELL = 1500; //TUNEME IN RPM
    public static double FLYWHEEL_SPEED_FROM_FLOWER = 2000; //TUNEME IN RPM


}
