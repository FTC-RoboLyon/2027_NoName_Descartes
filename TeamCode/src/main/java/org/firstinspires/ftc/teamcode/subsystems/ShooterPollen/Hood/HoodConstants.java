package org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Hood;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

public class HoodConstants {

    public static final ControlMode MainConntrolMode = ControlMode.POSITION_DUTYCYCLE_PID;
    public static final ControlMode EmergencyConntrolMode = ControlMode.MANUAL_POSITION;

    public static final String HOOD_SERVO_NAME = "hood";
    public static final String HOOD_ENCODEUR_NAME = "hoodEncodeur";
    public static final double HOOD_ENCODEUR_TICK_PER_ROTATION = 8192;
    public static final double HOOD_TICK_PER_RADIANT = (double) 139_264/(6*Math.PI);
    public static final double HOOD_NOMINAL_VOLTAGE = 8 ;

    public static double HOOD_ANGLE_NEAR = 1.0; //TUNEME IN RADIANT
    public static double HOOD_ANGLE_FACED_WITH_CELL = 1.0; //TUNEME IN RADIANT
    public static double HOOD_ANGLE_FROM_FLOWER = 1.0; //TUNEME IN RADIANT

    public static double HOOD_DEFAULT_POS = 1.0; //TUNEME IN RADIANT

    public static double HOOD_ANGLE_MIN = 0.5235987755982988; //TUNEME IN RADIANT
    public static double HOOD_ANGLE_MAX = 0.75049157835756; //TUNEME IN RADIANT

    public static double HOOD_KP = 0.0; // TUNEME //from radiant to dutyCycle
    public static double HOOD_KI = 0.0; // TUNEME //from radiant to dutyCycle
    public static double HOOD_KD = 0.0; // TUNEME //from radiant to dutyCycle

    public static double HOOD_TOLERANCE = 0.0; // TUNEME in RPM

}
