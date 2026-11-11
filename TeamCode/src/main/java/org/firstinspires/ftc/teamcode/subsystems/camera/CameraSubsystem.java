package org.firstinspires.ftc.teamcode.subsystems.camera;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.localization.LocalizationLimelight;

public class CameraSubsystem extends SubsystemBase {
    private LocalizationLimelight limelight;
    private RealtimeLogger logger;
    private RobotContainer robot;

    public CameraSubsystem(LocalizationLimelight limelight, RobotContainer robot, RealtimeLogger logger)
    {
        this.limelight = limelight;
        this.logger = logger;
        this.robot = robot;
    }

    @Override
    public void periodic()
    {
        logger.log("MT1 cam pos", limelight.Update().toArray());
        logger.log("MT2 cam pos", limelight.UpdateWithMT2(robot.getRobotPose().headingRadians).toArray());
    }
}
