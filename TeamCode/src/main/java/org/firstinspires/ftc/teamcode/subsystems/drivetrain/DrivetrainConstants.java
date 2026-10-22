package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;

/**
 * Physical and tuning constants for the mecanum drive. SI units throughout.
 *
 * <p>Geometry/speed are robot-specific (measure them). Device names must match
 *  * the Robot Configuration on the Control Hub.
 */
@Config
public final class DrivetrainConstants
{
    private DrivetrainConstants() {}

    // ---- Geometry  ----
    public static final double WHEEL_RADIUS_METERS = 0.052; // 104 mm mecanum wheel -> 52 mm radius
    public static final double WHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double DRIVE_GEAR_REDUCTION = 19.2; // motor rotations per wheel rotation
    public static final double TRACK_WIDTH_METERS = 0.4137; // left-right distance between wheel centers
    public static final double WHEEL_BASE_METERS = 0.336; // front-back distance between wheel centers

    public static final double TICKS_PER_WHEEL_REV = 537.7; // e.g. goBILDA 19.2:1 yellow jacket

    // ---- Speed limits ----
    public static final double MAX_GEARBOX_SPEED_RPS = 3.9; // 3.9 rot/s (motor) = 234 RPM (75% of the motor free speed)

    //  ~= 312 RPM = 5.2 rot/s (motor).
    public static final double MAX_WHEEL_SPEED_MPS = (MAX_GEARBOX_SPEED_RPS) * WHEEL_CIRCUMFERENCE_METERS;
    public static final double MAX_LINEAR_SPEED_MPS = MAX_WHEEL_SPEED_MPS; // forward/strafe teleop cap
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =
            MAX_WHEEL_SPEED_MPS / ((TRACK_WIDTH_METERS / 2.0) + (WHEEL_BASE_METERS / 2.0));

    // ---- Control ----
    // false = open-loop setPower (robust, no tuning); true = closed-loop setVelocity (needs encoders).
    public static final boolean USE_CLOSED_LOOP = false;
    public static final double NOMINAL_VOLTAGE = 12.0; // used only for the setVoltage() estimate
    public static double MANUAL_SETPOINT = 0.12;

    // ---- Device names (must match the Control Hub Robot Configuration) ----
    public static final String FRONT_LEFT_NAME = "frontLeft";
    public static final String FRONT_RIGHT_NAME = "frontRight";
    public static final String REAR_LEFT_NAME = "backLeft";
    public static final String REAR_RIGHT_NAME = "backRight";

    // ---- Teleop feel ----
    public static final double JOYSTICK_DEADBAND = 0.1;

    // ---- Reversals (right side typically reversed so +X = forward) ----
    public static final boolean FRONT_LEFT_REVERSED = false;
    public static final boolean FRONT_RIGHT_REVERSED = true;
    public static final boolean REAR_LEFT_REVERSED = false;
    public static final boolean REAR_RIGHT_REVERSED = true;


    // ---- PID Control ----
    public static final double DISTANCE_KP = 1.15;
    public static final double DISTANCE_KI = 0.0;
    public static final double DISTANCE_KD = 0.0;

    public static double HEADING_KP = 10.0;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.37;

    //static-friction feedforward go to pose
    public static final double STATIC_FRICTION_DISTANCE_M = 0.012;
    public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.05;

    public static final double PATH_X_KP = 0.0;
    public static final double PATH_Y_KP = 0.0;

    //Wheel feedforward
    public static final double WHEEL_KS = 0.0;                       // static friction
    public static final double WHEEL_KV = 1.0 / MAX_WHEEL_SPEED_MPS;
    public static final double WHEEL_KA = 0.0;


    // ---- Odometry ----
    public static final String PINPOINT_NAME = "odoPinpoint";
    public static final Pose2d PINPOINT_OFFSETS = new Pose2d(0.0,0.0,0.0);
    public static final double PINPOINT_YAW_SCALAR = 0.9997;

    //1st pod
    public static final Pose2d POD1_OFFSETS = new Pose2d(-192.6085e-3, 4.0e-3, Math.toRadians(0.0));
    public static final double POD1_WHEEL_RADIUS = 16e-3; //in m
    public static final double POD1_ENCODER_RESOLUTION = 2000; //in CPR
    public static final boolean POD1_ENCODER_REVERSED = true;

    //2nd pod
    public static final Pose2d POD2_OFFSETS = new Pose2d(0.591430e-3, -189.2e-3, Math.toRadians(90.0));
    public static final double POD2_WHEEL_RADIUS = 16e-3; //in m
    public static final double POD2_ENCODER_RESOLUTION = 2000; //in CPR
    public static final boolean POD2_ENCODER_REVERSED = false;

    // ---- OTOS ----
    public static final String OTOS_NAME = "otos";
    public static final double OTOS_X_OFFSET = 43e-3; //in m
    public static final double OTOS_Y_OFFSET = -6e-3; //in m
    public static final double OTOS_HEADING_OFFSET = -(Math.PI/2); //in rad
    public static final double OTOS_ANGULAR_SCALAR = 0.995;


    // ---- Drivetrain Mathematical Model ----
    public static final Pose2d MODEL_PROCESS_NOISE = new Pose2d(0.01,0.01,Math.PI/180); //TUNEME

    // ---- Wheel Coefs ----
    public static double BACK_LEFT_COEF = 0.85;
    public static double FRONT_LEFT_COEF = 0.85;
    public static double BACK_RIGHT_COEF = 0.85;
    public static double FRONT_RIGHT_COEF = 1.0;
}



//TODO : see tank drivetrain
//TODO : add wheels positions