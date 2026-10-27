package org.firstinspires.ftc.teamcode.subsystems.ShooterParametersCalculator;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import java.util.function.Supplier;

public class ShooterParametersCalculator {
    private Supplier<Pose2d> getRobotPos;
    private Supplier<Pose2d> getRobotSpeed;
    private Pose2d targetPos = new Pose2d();
    public void setTargetPos(Pose2d pos){targetPos = pos;}
    public Pose2d getTargetPos(){return targetPos;}

    private Telemetry telemetry;
    public ShooterParametersCalculator(Supplier<Pose2d> position, Telemetry telemetry)
    {
        getRobotPos = position;
        this.telemetry = telemetry;
    }


    public void update()
    {

    }

    private void calculateXspeed()
    {
        //for future
    }
    private void calculateYspeed()
    {
        //for future
    }
    private void calculateAngularSpeed()
    {
        // for future
    }


    public double getFlywheelSpeedForPollen(Pose2d targetPos)
    {
        //TODO write and tune a way to find Shooter needed speed to reach a target (by extrapolation or calculus)
        return 0.0;
    }

    public double getHoodAngleForPollen(Pose2d targetPos)
    {
        //TODO write and tune a way to find Hood needed Angle to reach a target (by extrapolation or calculus)
        return 0.0;
    }

    public double getFlywheelSpeedForNectar(Pose2d targetPos)
    {
        //TODO write and tune a way to find Shooter needed speed to reach a target (by extrapolation or calculus)
        return 0.0;
    }

    public double getHoodAngleForNectar(Pose2d targetPos)
    {
        //TODO write and tune a way to find Hood needed Angle to reach a target (by extrapolation or calculus)
        return 0.0;
    }

    public double getTurretRadiantTargetWithTargetPosition(Pose2d targetPos1) {
        double xRobot = getRobotPos.get().xMeters;
        double yRobot = getRobotPos.get().yMeters;
        double headingRobot = getRobotPos.get().headingRadians;

        double xTargetRobot = Math.cos(headingRobot - TurretConstants.offsetHeading) * (targetPos1.xMeters - xRobot)
                + Math.sin(headingRobot - TurretConstants.offsetHeading) * (targetPos1.yMeters - yRobot);

        double yTargetRobot = -Math.sin(headingRobot - TurretConstants.offsetHeading) * (targetPos1.xMeters - xRobot)
                + Math.cos(headingRobot - TurretConstants.offsetHeading) * (targetPos1.yMeters - yRobot);

        return Math.atan2(yTargetRobot, xTargetRobot);
    }
}
