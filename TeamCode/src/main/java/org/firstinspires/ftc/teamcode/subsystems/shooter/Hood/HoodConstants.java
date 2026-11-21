package org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;
import org.firstinspires.ftc.LyonLib.control.ControlMode;

public class HoodConstants {

    public static final ControlMode MainConntrolMode = ControlMode.POSITION_DUTYCYCLE_PID;
    public static final ControlMode EmergencyConntrolMode = ControlMode.MANUAL_POSITION;

    public static final String HOOD_SERVO_NAME = "hood";
    public static final String HOOD_ENCODEUR_NAME = "hoodEncodeur";
    public static final double HOOD_ENCODEUR_TICK_PER_ROTATION = 8192;
    public static final double HOOD_TICK_PER_RADIANT = (double) 139_264/(6*Math.PI);
    public static final double HOOD_NOMINAL_VOLTAGE = 8 ;

    public static double HOOD_ANGLE_NEAR = 0.0; //TUNEME IN RADIANT
    public static double HOOD_ANGLE_FACED_WITH_CELL = 0.5; //TUNEME IN RADIANT
    public static double HOOD_ANGLE_FROM_FLOWER = 1.0; //TUNEME IN RADIANT

    public static double HOOD_DEFAULT_POS = 0.5; //TUNEME IN RADIANT

    public static double HOOD_ANGLE_MIN = Math.toRadians(30); //TUNEME IN DEGREES
    public static double HOOD_ANGLE_MAX = Math.toRadians(43); //TUNEME DEGREES
    public static double HOOD_RANGE = HOOD_ANGLE_MAX - HOOD_ANGLE_MIN; // = 0.2268928027592612 RADIANT = 13 degrees

    public static double HOOD_KP = 0.0; // TUNEME //from radiant to dutyCycle
    public static double HOOD_KI = 0.0; // TUNEME //from radiant to dutyCycle
    public static double HOOD_KD = 0.0; // TUNEME //from radiant to dutyCycle


    public static double HOOD_VELOCITY_COMPENSATION = 0.0; // TUNEME will be multiplied the flywheel speed error to be add to the hood target angle

    public static double HOOD_NEAR_POS = 0.0; //TUNEME
    public static double HOOD_ALIGNED_WITH_CELL_POS = 0.0; //TUNEME
    public static double HOOD_FROM_FLOWER_POS = 0.0; //TUNEME

    public static double HOOD_TOLERANCE = 0.0; // TUNEME in RPM

}
