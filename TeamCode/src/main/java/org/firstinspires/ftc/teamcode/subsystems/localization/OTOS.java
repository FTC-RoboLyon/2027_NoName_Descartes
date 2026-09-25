package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.LocalizationSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OTOS implements LocalizationSensor {
    private final SparkFunOTOS otos;
    private Pose2d EstimatedPos = new Pose2d();
    private boolean isInit = false;
    private final String deviceName;

    private Pose2d StdDev;


    public OTOS(HardwareMap hmap, String deviceName, double xOffset, double yOffset, double headingOffset, Pose2d StdDev, double AngularScalar) {
        otos = hmap.get(SparkFunOTOS.class, deviceName);

        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(new SparkFunOTOS.Pose2D(xOffset, yOffset, headingOffset));
        otos.setAngularScalar(AngularScalar);

        this.StdDev = StdDev;
        this.deviceName = deviceName;
    }

    @Override
    public void Init(Pose2d startingPos) {
        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(startingPos.xMeters, startingPos.yMeters, startingPos.headingRadians));
        EstimatedPos = startingPos;
        isInit = true;
    }

    @Override
    public void SetPosition(Pose2d pos) {
        otos.setPosition(new SparkFunOTOS.Pose2D(pos.xMeters, pos.yMeters, pos.headingRadians));
        EstimatedPos = pos;
    }

    @Override
    public Pose2d Update()
    {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        EstimatedPos = new Pose2d(pos.x, pos.y, pos.h);
        return EstimatedPos;
    }

    @Override
    public boolean IsReady()
    {
        return otos.isConnected() && isInit;
    }

    @Override
    public Pose2d GetLastEstimatedPos()
    {
        return EstimatedPos;
    }

    @Override
    public Pose2d GetStdDev() {
        return StdDev;
    }

    @Override
    public Pose2d GetAnalysedAxes() {return new Pose2d(1,1,1);}

    @Override
    public String GetSensorName()
    {
        return deviceName;
    }

    public double GetAngularScalar()
    {
        return otos.getAngularScalar();
    }

    public void SetAngularScalar(double AngularScalar)
    {
        otos.setAngularScalar(AngularScalar);
    }
}