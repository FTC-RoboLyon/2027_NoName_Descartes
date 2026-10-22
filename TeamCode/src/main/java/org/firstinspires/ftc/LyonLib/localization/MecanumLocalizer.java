package org.firstinspires.ftc.LyonLib.localization;

import org.firstinspires.ftc.LyonLib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.LyonLib.kinematics.MecanumWheelSpeeds;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;

public class MecanumLocalizer extends RobotLocalizer{

    public MecanumLocalizer() {super();};

    public MecanumLocalizer(RealtimeLogger logger) {super(logger);}

    protected Pose2d CalculatePosWithModel(double timestamp)
    {
        ChassisSpeeds chassisSpeeds = drivetrainKinematics.toChassisSpeeds(new MecanumWheelSpeeds(wheels[0].velocityMetersPerSecond,
                                                                                                  wheels[1].velocityMetersPerSecond,
                                                                                                  wheels[2].velocityMetersPerSecond,
                                                                                                  wheels[3].velocityMetersPerSecond));
        double dt = timestamp - lastTimestamp;

        double forwardDisplacement = chassisSpeeds.vx*dt;
        double strafeDisplacement = chassisSpeeds.vy*dt;
        double rotation = chassisSpeeds.omega*dt;

        lastTimestamp = timestamp;
        return EstimatedPos.exp(forwardDisplacement, strafeDisplacement, rotation);
    }
}