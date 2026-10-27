package org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;

public class FlywheelConstants {
    public static final String FLYWHEEL_MOTOR1_NAME = "flywheelR";
    public static final String FLYWHEEL_MOTOR2_NAME = "flywheelL";

    public static double FLYWHEEL_KP = 0.0; //TUNEME
    public static double FLYWHEEL_KI = 0.0;//TUNEME
    public static double FLYWHEEL_KD = 0.0;//TUNEME

    public static double FLYWHEEL_KV = 0.0; //TUNEME
    public static double FLYWHEEL_KA = 0.0;//TUNEME
    public static double FLYWHEEL_KS = 0.0;//TUNEME
    public static double FLYWHEEL_TOLERANCE = 50.0;//TUNEME

    public static double FLYWHEEL_TARGET_SPEED_POLLEN_NEAR = 0.0; //TUNEME
    public static double FLYWHEEL_TARGET_SPEED_POLLEN_MID = 0.0; //TUNEME
    public static double FLYWHEEL_TARGET_SPEED_POLLEN_FAR = 0.0; //TUNEME

    public static double FLYWHEEL_TARGET_SPEED_NECTAR_NEAR = 0.0; //TUNEME
    public static double FLYWHEEL_TARGET_SPEED_NECTAR_MID = 0.0; //TUNEME
    public static double FLYWHEEL_TARGET_SPEED_NECTAR_FAR = 0.0; //TUNEME

    public static final double FLYWHEEL_GEAR_RATIO = 3; //for both
    public static final double FLYWHEEL_TICKS_PER_ROTATION = 28*FLYWHEEL_GEAR_RATIO;

    public static final double FLYWHEEL_NOMINAL_VOLTAGE = 12;

}
