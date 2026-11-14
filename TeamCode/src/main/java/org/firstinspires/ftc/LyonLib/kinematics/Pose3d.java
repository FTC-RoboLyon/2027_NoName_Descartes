package org.firstinspires.ftc.LyonLib.kinematics;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Pose3d {
    // Position :
    public double xMeters;
    public double yMeters;
    public double zMeters;

    // Orientation :
    public double yawRadians;
    public double pitchRadians;
    public double rollRadians;

    public Pose3d()
    {
        this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public Pose3d(double xMeters, double yMeters, double zMeters, double yawRadians, double pitchRadians, double rollRadians)
    {
        this.xMeters = xMeters;
        this.yMeters = yMeters;
        this.zMeters = zMeters;

        this.yawRadians = yawRadians;
        this.pitchRadians = pitchRadians;
        this.rollRadians = rollRadians;
    }

    public Pose3d(Pose2d PositionIn2d, double zMeters, double pitchRadians, double rollRadians)
    {
        this.xMeters = PositionIn2d.xMeters;
        this.yMeters = PositionIn2d.yMeters;
        this.yawRadians = PositionIn2d.headingRadians;

        this.zMeters = zMeters;
        this.pitchRadians = pitchRadians;
        this.rollRadians = rollRadians; 
    }  

    public Pose3d(Pose3D pos)
    {
        this.xMeters = pos.getPosition().x;
        this.yMeters = pos.getPosition().y;
        this.zMeters = pos.getPosition().z;

        this.yawRadians = pos.getOrientation().getYaw(AngleUnit.RADIANS);
        this.pitchRadians = pos.getOrientation().getPitch(AngleUnit.RADIANS);
        this.rollRadians = pos.getOrientation().getRoll(AngleUnit.RADIANS);
    }

    public Pose2d toPose2d()
    {
        return new Pose2d(xMeters, yMeters, yawRadians);
    }

    public Pose3d copy()
    {
        return new Pose3d(xMeters, yMeters, zMeters, yawRadians, pitchRadians, rollRadians);
    }
}