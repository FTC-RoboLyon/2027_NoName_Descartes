package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.LocalizationSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LocalizationLimelight implements LocalizationSensor {
    private final Limelight3A camera;

    private LLResult latestResult;

    private Pose2d EstimatedPos;

    private boolean PosEstimatedWithMT2 = false;
    private final String deviceName;

    public LocalizationLimelight(HardwareMap hmap, String deviceName)
    {
        this.camera = hmap.get(Limelight3A.class, deviceName);
        camera.setPollRateHz(100);
        this.deviceName = deviceName;
    }

    @Override
    public void Init(Pose2d startingPos)
    {
        camera.start();
    }

    /**
     * Update EstimatedPos with the latest camera result with the MegaTag1 algorithm
     * in order to be totally independent.
     * If you want to use the MegaTag2 algorithm you
     * must use {@link LocalizationLimelight#UpdateWithMT2(double robotYaw)}.
     * @return the estimated robot position with the MegaTag1 algorithm
     */
    @Override
    public Pose2d Update()
    {
        PosEstimatedWithMT2 = false;
        latestResult = camera.getLatestResult();
        if (latestResult.isValid())
        {
            Pose3D botPos = latestResult.getBotpose();
            EstimatedPos = new Pose2d(botPos.getPosition().x, botPos.getPosition().y, botPos.getOrientation().getYaw(AngleUnit.RADIANS));
        }
        else
        {
            EstimatedPos = new Pose2d();
        }
        return EstimatedPos;
    }

    /**
     * Update EstimatedPos with the latest camera result with the MegaTag2 algorithm.
     * This method is not inherited from LocalisationSensor interface.
     * @return the estimated robot position with the MegaTag2 algorithm
     */
    public Pose2d UpdateWithMT2(double robotYaw)
    {
        PosEstimatedWithMT2 = true;
        camera.updateRobotOrientation(robotYaw);
        latestResult = camera.getLatestResult();
        if (latestResult.isValid())
        {
            Pose3D botPos = latestResult.getBotpose_MT2();
            EstimatedPos = new Pose2d(botPos.getPosition().x, botPos.getPosition().y, botPos.getOrientation().getYaw(AngleUnit.RADIANS));
        }
        else
        {
            EstimatedPos = new Pose2d();
        }
        return EstimatedPos;
    }

    @Override
    public Pose2d GetLastEstimatedPos()
    {
        return EstimatedPos;
    }

    @Override
    public boolean IsReady()
    {
        return camera.isConnected() && camera.isRunning();
    }

    @Override
    public Pose2d GetAnalysedAxes()
    {
        if (PosEstimatedWithMT2)
        {
            return new Pose2d(1,1,0);
        }
        else
        {
            return new Pose2d(1,1,1);
        }
    }

    @Override
    public Pose2d GetStdDev()
    {
        if(latestResult.isValid())
        {
            if (PosEstimatedWithMT2)
            {
                double[] stdDev = latestResult.getStddevMt2();
                return new Pose2d(stdDev[0],stdDev[1],1e10);
            }
            else
            {
                double[] stdDev = latestResult.getStddevMt1();
                return new Pose2d(stdDev[0],stdDev[1],stdDev[5]);
            }
        }
        return new Pose2d(1e9,1e9,1e9);
    }

    @Override
    public String GetSensorName()
    {
        return deviceName;
    }
}
