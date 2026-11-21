package org.firstinspires.ftc.teamcode.subsystems.localization;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;

public class localizationConstants {

    // --- Hive ---
    public static final Pose2d RedCentreHive = new Pose2d(1.79085,1.46719,0.0);
    public static final Pose2d BlueCentreHive = new Pose2d( 2.11489,1.46719,0.0);
    public static final Pose2d DiffWhenLow = new Pose2d(2.4e-5,2.894781e-1,0.0);
    public static final Pose2d DiffWhenHigh = new Pose2d(2.4e-5,0.3274046,0.0);

    public static final double publicLauchZone = 0.0; // TUNEME //i couldn't find a better zone but this design the y value from which the shoot isn't possible anymore (because robot is either under the hive either on its side)
    public static final double oppositeLauchZone = 0.0;//TUNEME //same for the other side

    public static final Pose2d bluePublicCellePos = new Pose2d(localizationConstants.BlueCentreHive.xMeters, localizationConstants.BlueCentreHive.yMeters - localizationConstants.DiffWhenHigh.yMeters,0.0 ),
            blueOppositeCellPos = new Pose2d(localizationConstants.BlueCentreHive.xMeters, localizationConstants.BlueCentreHive.yMeters + localizationConstants.DiffWhenHigh.yMeters,0.0 ),
            redPublicCellPos = new Pose2d(localizationConstants.RedCentreHive.xMeters, localizationConstants.RedCentreHive.yMeters - localizationConstants.DiffWhenHigh.yMeters, 0.0),
            redOppositeCellPos= new Pose2d(localizationConstants.RedCentreHive.xMeters, localizationConstants.RedCentreHive.yMeters + localizationConstants.DiffWhenHigh.yMeters, 0.0);
}
