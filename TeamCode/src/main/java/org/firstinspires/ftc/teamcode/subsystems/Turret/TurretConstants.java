package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.localization.localizationConstants;
@Config
public class TurretConstants {
    public static final String TURRET_NAME = "turret";
    public static final String HALL_SENSOR_NAME = "hallSensor";
    public static final Pose2d bluePublicCellePos = new Pose2d(localizationConstants.BlueCentreHive.xMeters, localizationConstants.BlueCentreHive.yMeters - localizationConstants.DiffWhenHigh.yMeters,0.0 ),
            blueOppositeCellPos = new Pose2d(localizationConstants.BlueCentreHive.xMeters, localizationConstants.BlueCentreHive.yMeters + localizationConstants.DiffWhenHigh.yMeters,0.0 ),
            redPublicCellPos = new Pose2d(localizationConstants.RedCentreHive.xMeters, localizationConstants.RedCentreHive.yMeters - localizationConstants.DiffWhenHigh.yMeters, 0.0),
            redOppositeCellPos= new Pose2d(localizationConstants.RedCentreHive.xMeters, localizationConstants.RedCentreHive.yMeters + localizationConstants.DiffWhenHigh.yMeters, 0.0);
    public static Pose2d publicSideCenter = new Pose2d(); //TUNEME
    public static Pose2d oppositeSideCenter = new Pose2d(); //TUNEME

    public static double offsetHeading = 0.0;//TUNEME

    public static double turretKP = 0.00025;//TUNEME
    public static double turretKI = 0.0;//TUNEME

    public static double turretKD = 0.004;//TUNEME

    public static double turretTolerance = 100;//TUNEME in ticks
    public static double hallSensorMinWhenMagnet = 1; //TUNEME in voltage
    public static double hallSensorTolerance = 1; //TUNEME in voltage

    @Deprecated
    public final static double TICKS_PER_DEGREES = (double) 32768 /405;
    public final static double TICKS_PER_RADIANT = (double) 1048576/(70 * Math.PI);

    public final static double TURRET_NOMINAL_VOLTAGE = 11.5;


}
