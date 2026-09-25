package org.firstinspires.ftc.teamcode.subsystems.localization;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.LocalizationSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class OdometryWithPinpoint implements LocalizationSensor {
    private final GoBildaPinpointDriver odometryPinpoint;
    private final OdometryPod Pod1, Pod2;

    private Pose2d EstimatedPos = new Pose2d();
    private Pose2d StdDev;
    private int Pod1LastValue = 0;
    private int Pod2LastValue = 0;
    private boolean isInit = false;
    private final String deviceName;


    public OdometryWithPinpoint(HardwareMap hmap, String deviceName, OdometryPod Pod1, OdometryPod Pod2, Pose2d StdDev, double AngularScalar)
    {
        odometryPinpoint = hmap.get(GoBildaPinpointDriver.class, deviceName);
        odometryPinpoint.setYawScalar(AngularScalar);
        this.Pod1= Pod1;
        this.Pod2 = Pod2;
        this.StdDev = StdDev;
        this.deviceName = deviceName;
    }

    @Override
    public void Init(Pose2d startingPos)
    {
        //Configure parameters for the pinpoint (normally useless because we will do our own calculations but just in case)
        odometryPinpoint.setOffsets(Pod1.yOffset, Pod2.xOffset, DistanceUnit.METER);
        odometryPinpoint.setEncoderResolution((Pod1.encoderResolution)/(2*Pod1.wheelRadius*Math.PI), DistanceUnit.METER);
        odometryPinpoint.setEncoderDirections(Pod1.reverseEncoder? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Pod2.reverseEncoder? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //Reset Pinpoint IMU and save starting encoder value
        odometryPinpoint.resetPosAndIMU();
        odometryPinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.METER,startingPos.xMeters,startingPos.yMeters,
                AngleUnit.RADIANS, startingPos.headingRadians));
        EstimatedPos = startingPos;

        odometryPinpoint.update();
        Pod1LastValue = odometryPinpoint.getEncoderX();
        if (Pod1.reverseEncoder)
        {
            Pod1LastValue = -Pod1LastValue;
        }

        Pod2LastValue = odometryPinpoint.getEncoderY();
        if (Pod2.reverseEncoder)
        {
            Pod2LastValue = -Pod2LastValue;
        }
        isInit = true;
    }

    @Override
    public void SetPosition(Pose2d pos)
    {
        EstimatedPos = pos;
        odometryPinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.METER,pos.xMeters,pos.yMeters,
                AngleUnit.RADIANS, pos.headingRadians));
    }

    @Override
    public Pose2d Update()
    {
        //Get sensor values reversing values if needed
        odometryPinpoint.update();
        double newHeading = odometryPinpoint.getHeading(AngleUnit.RADIANS);

        int Pod1Value = odometryPinpoint.getEncoderX();
        if (Pod1.reverseEncoder)
        {
            Pod1Value = -Pod1Value;
        }

        int Pod2Value = odometryPinpoint.getEncoderY();
        if (Pod2.reverseEncoder)
        {
            Pod2Value = -Pod2Value;
        }

        //Calculate deltas
        double dHeading = newHeading - EstimatedPos.headingRadians;

        //Prevent errors when the heading passes from PI to minus PI
        if (dHeading > Math.PI)
        {
            dHeading -= 2*Math.PI;
        }
        if (dHeading < -Math.PI)
        {
            dHeading += 2*Math.PI;
        }

        double dPod1 = Pod1Value - Pod1LastValue;
        double dPod2 = Pod2Value - Pod2LastValue;

        //Convert values in meters and correct errors caused by offsets
        double CorrectedPod1Value = dPod1*Pod1.metersPerCount - dHeading*Pod1.headingCorrectionFactor;
        double CorrectedPod2Value = dPod2*Pod2.metersPerCount - dHeading*Pod2.headingCorrectionFactor;

        //Calculates displacements in the robot coordinates
        double forward = Math.cos(Pod1.headingOffset)*CorrectedPod1Value + Math.cos(Pod2.headingOffset)*CorrectedPod2Value;
        double strafe = Math.sin(Pod1.headingOffset)*CorrectedPod1Value + Math.sin(Pod2.headingOffset)*CorrectedPod2Value;

        //Applies these movements in the field coordinates
        double midHeading = EstimatedPos.headingRadians+dHeading/2.0;
        double dX = Math.cos(midHeading)*forward - Math.sin(midHeading)*strafe;
        double dY = Math.sin(midHeading)*forward + Math.cos(midHeading)*strafe;


        EstimatedPos.headingRadians = newHeading;
        EstimatedPos.xMeters += dX;
        EstimatedPos.yMeters += dY;

        //Save the encoder values for the next loop
        Pod1LastValue = Pod1Value;
        Pod2LastValue = Pod2Value;

        //Return the estimated pos
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
        return odometryPinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY && isInit;
    }

    @Override
    public Pose2d GetStdDev()
    {
        return StdDev;
    }

    @Override
    public Pose2d GetAnalysedAxes() {return new Pose2d(1,1,1);}

    @Override
    public String GetSensorName()
    {
        return deviceName;
    }

    public GoBildaPinpointDriver.DeviceStatus GetDeviceStatus()
    {
        return odometryPinpoint.getDeviceStatus();
    }


    public void SetAngularScalar(double AngularScalar)
    {
        odometryPinpoint.setYawScalar(AngularScalar);
    }

    public float GetAngularScalar()
    {
        return odometryPinpoint.getYawScalar();
    }

    public static class OdometryPod
    {
        public double xOffset; //in m
        public double yOffset; //in m
        public double headingOffset; //in radians
        public double wheelRadius; //in m
        public double encoderResolution; //in CPR
        public boolean reverseEncoder;

        public double metersPerCount; //in m/Count
        public double headingCorrectionFactor; //in m

        public OdometryPod(double xOffset, double yOffset, double headingOffset, double wheelRadius, double encoderResolution, boolean reverseEncoder)
        {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.headingOffset = headingOffset;
            this.wheelRadius = wheelRadius;
            this.encoderResolution = encoderResolution;
            this.reverseEncoder = reverseEncoder;

            this.metersPerCount = 2*Math.PI*this.wheelRadius/this.encoderResolution;
            this.headingCorrectionFactor = xOffset*Math.sin(headingOffset) - yOffset*Math.cos(headingOffset);
        }
    }
}