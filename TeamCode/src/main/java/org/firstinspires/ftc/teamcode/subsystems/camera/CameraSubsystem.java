package org.firstinspires.ftc.teamcode.subsystems.camera;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.kinematics.Pose3d;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;

import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterParametersCalculator.ShooterParametersCalculator.ShootParameters;
import org.firstinspires.ftc.teamcode.subsystems.localization.LocalizationLimelight;

public class CameraSubsystem extends SubsystemBase {

    public enum VisionMode
    {
        DISABLED,
        TRACK_RED_HIVE,
        TRACK_BLUE_HIVE
    }

    private Limelight3A limelight;
    private LLResult latestResult;
    private VisionMode visionMode = VisionMode.DISABLED;

    private RealtimeLogger logger;
    private RobotContainer robot;
    private ShootParameters shootParams;

    public void SetVisionMode(VisionMode visionMode)
    {
        if (this.visionMode != visionMode)
        {
            switch(visionMode)
            {
                case DISABLED:
                    this.visionMode = visionMode;
                    break;

                case TRACK_RED_HIVE:
                    limelight.pipelineSwitch(CameraConstants.RED_PIPELINE_ID);
                    this.visionMode = visionMode;
                    break;

                case TRACK_BLUE_HIVE:
                    limelight.pipelineSwitch(CameraConstants.BLUE_PIPELINE_ID);
                    this.visionMode = visionMode;
                    break;

                default :
                    throw new IllegalArgumentException("I can't watch sth I don't know");
            }
        }
    }


    public CameraSubsystem(Limelight3A limelight, RobotContainer robot, RealtimeLogger logger, ShootParameters shootParams)
    {
        this.limelight = limelight;
        this.logger = logger;
        this.robot = robot;
        this.shootParams = shootParams;
    }

    @Override
    public void periodic()
    {
        if (visionMode == VisionMode.TRACK_RED_HIVE || visionMode == VisionMode.TRACK_BLUE_HIVE)
        {
            limelight.updateRobotOrientation(robot.getRobotPose().headingRadians);
        }

        latestResult = limelight.getLatestResult();

        if(latestResult.isValid())
        {
            switch(visionMode)
            {
                case DISABLED:
                    break;

                case TRACK_RED_HIVE:
                    if(latestResult.getBotposeSpan() >= 1)
                    {
                        Pose2d RobotInField = robot.getRobotPose();
                        Pose3d RobotInFrameFrame = new Pose3d(RobotInField.xMeters - Constants.RED_FRAME_CENTRE_POS.xMeters,
                                                              RobotInField.yMeters - Constants.RED_FRAME_CENTRE_POS.yMeters,
                                                              -Constants.FRAME_HEIGHT,
                                                              RobotInField.headingRadians, 0.0, 0.0);

                        Pose3d RobotInHive = new Pose3d(latestResult.getBotpose_MT2());
                
                        double HiveAngle = Math.atan2((RobotInFrameFrame.yMeters * RobotInHive.zMeters) - (RobotInHive.yMeters * RobotInFrameFrame.zMeters),
                                                       (RobotInFrameFrame.yMeters * RobotInHive.yMeters) - (RobotInFrameFrame.zMeters * RobotInHive.zMeters));

                        double cos = Math.cos(HiveAngle);
                        double sin = Math.sin(HiveAngle);

                        if(HiveAngle >= 0.0)
                        {
                            shootParams.targetPos = new Pose3d(Constants.RED_FRAME_CENTRE_POS.xMeters,
                                                               Constants.RED_FRAME_CENTRE_POS.yMeters + cos*-Constants.HIVE_ARM_LENGTH/2.0 + sin*(Constants.CELL_HEIGHT-Constants.PIVOT_HEIGHT_IN_HIVE),
                                                               Constants.FRAME_HEIGHT - sin*-Constants.HIVE_ARM_LENGTH/2.0 + cos*(Constants.CELL_HEIGHT-Constants.PIVOT_HEIGHT_IN_HIVE),
                                                               0.0,0.0,0.0);
                        }
                        else
                        {
                            shootParams.targetPos = new Pose3d(Constants.RED_FRAME_CENTRE_POS.xMeters,
                                                               Constants.RED_FRAME_CENTRE_POS.yMeters + cos*Constants.HIVE_ARM_LENGTH/2.0 + sin*(Constants.CELL_HEIGHT-Constants.PIVOT_HEIGHT_IN_HIVE),
                                                               Constants.FRAME_HEIGHT - sin*Constants.HIVE_ARM_LENGTH/2.0 + cos*(Constants.CELL_HEIGHT-Constants.PIVOT_HEIGHT_IN_HIVE),
                                                               0.0,0.0,0.0);
                        }                          
                    }
                    else
                    {
                        //TODO
                    }
                    break;

                case TRACK_BLUE_HIVE:
                    if(latestResult.getBotposeSpan() >= 1)
                    {
                        Pose2d RobotInField = robot.getRobotPose();
                        Pose3d RobotInFrameFrame = new Pose3d(RobotInField.xMeters - Constants.BLUE_FRAME_CENTRE_POS.xMeters,
                                                              RobotInField.yMeters - Constants.BLUE_FRAME_CENTRE_POS.yMeters,
                                                              -Constants.FRAME_HEIGHT,
                                                              RobotInField.headingRadians, 0.0, 0.0);

                        Pose3d RobotInHive = new Pose3d(latestResult.getBotpose_MT2());
                
                        double HiveAngle = Math.atan2((RobotInFrameFrame.yMeters * RobotInHive.zMeters) - (RobotInHive.yMeters * RobotInFrameFrame.zMeters),
                                                       (RobotInFrameFrame.yMeters * RobotInHive.yMeters) - (RobotInFrameFrame.zMeters * RobotInHive.zMeters));

                        double cos = Math.cos(HiveAngle);
                        double sin = Math.sin(HiveAngle);

                        if(HiveAngle >= 0.0)
                        {
                            shootParams.targetPos = new Pose3d(Constants.BLUE_FRAME_CENTRE_POS.xMeters,
                                                               Constants.BLUE_FRAME_CENTRE_POS.yMeters + cos*-Constants.HIVE_ARM_LENGTH/2.0 + sin*(Constants.CELL_HEIGHT-Constants.PIVOT_HEIGHT_IN_HIVE),
                                                               Constants.FRAME_HEIGHT - sin*-Constants.HIVE_ARM_LENGTH/2.0 + cos*(Constants.CELL_HEIGHT-Constants.PIVOT_HEIGHT_IN_HIVE),
                                                               0.0,0.0,0.0);
                        }
                        else
                        {
                            shootParams.targetPos = new Pose3d(Constants.BLUE_FRAME_CENTRE_POS.xMeters,
                                                               Constants.BLUE_FRAME_CENTRE_POS.yMeters + cos*Constants.HIVE_ARM_LENGTH/2.0 + sin*(Constants.CELL_HEIGHT-Constants.PIVOT_HEIGHT_IN_HIVE),
                                                               Constants.FRAME_HEIGHT - sin*Constants.HIVE_ARM_LENGTH/2.0 + cos*(Constants.CELL_HEIGHT-Constants.PIVOT_HEIGHT_IN_HIVE),
                                                               0.0,0.0,0.0);
                        }       
                    }
                    else
                    {
                        //TODO
                    }
                    break;
            }
        }
        else
        {
            robot.getTelemetry().addLine("Limelight has closed its eye");
        }
    }
}
