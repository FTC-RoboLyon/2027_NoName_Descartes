package org.firstinspires.ftc.teamcode.subsystems.ShooterParametersCalculator;

import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooterNectarPollen.Hood.HoodConstants;

import java.util.function.Supplier;

public class ShooterParametersCalculator {
    private Telemetry telemetry;

    double xRobot;
    double yRobot;
    double headingRobot;
    private double distanceToTarget = 0.0;

    private Supplier<Pose2d> getRobotPos;
    //private Pose2d RobotSpeed;
    private Pose2d targetPos = new Pose2d();

    public void setTargetPos(Pose2d pos){targetPos = pos;}
    public Pose2d getTargetPos(){return targetPos;}

    private InterpLUT flywheelInterpolator = new InterpLUT();
    private InterpLUT hoodInterpolator = new InterpLUT();

    public class ShootParam
    {
        public double flywheelSpeed = 0.0;
        public double hoodAngle = HoodConstants.HOOD_DEFAULT_POS;
        public double targetTurretAngle = 0.0;
    }

    private ShootParam shootParam;
    public ShootParam getShootParam()
    {
        if (shootParam == null)
            shootParam = new ShootParam();
        return shootParam;
    }

    public ShooterParametersCalculator(Supplier<Pose2d> position, Telemetry telemetry)
    {
        getRobotPos = position;
        this.telemetry = telemetry;

        flywheelInterpolator.add(ShooterParametersConstants.input1, ShooterParametersConstants.flywheelSpeed1);
        flywheelInterpolator.add(ShooterParametersConstants.input2, ShooterParametersConstants.flywheelSpeed2);
        flywheelInterpolator.add(ShooterParametersConstants.input3, ShooterParametersConstants.flywheelSpeed3);
        flywheelInterpolator.add(ShooterParametersConstants.input4, ShooterParametersConstants.flywheelSpeed4);
        flywheelInterpolator.add(ShooterParametersConstants.input5, ShooterParametersConstants.flywheelSpeed5);
        flywheelInterpolator.add(ShooterParametersConstants.input6, ShooterParametersConstants.flywheelSpeed6);
        flywheelInterpolator.add(ShooterParametersConstants.input7, ShooterParametersConstants.flywheelSpeed7);
        flywheelInterpolator.add(ShooterParametersConstants.input8, ShooterParametersConstants.flywheelSpeed8);
        flywheelInterpolator.add(ShooterParametersConstants.input9, ShooterParametersConstants.flywheelSpeed9);
        flywheelInterpolator.add(ShooterParametersConstants.input10, ShooterParametersConstants.flywheelSpeed10);
        flywheelInterpolator.add(ShooterParametersConstants.input11, ShooterParametersConstants.flywheelSpeed11);
        flywheelInterpolator.add(ShooterParametersConstants.input12, ShooterParametersConstants.flywheelSpeed12);
        flywheelInterpolator.add(ShooterParametersConstants.input13, ShooterParametersConstants.flywheelSpeed13);
        flywheelInterpolator.add(ShooterParametersConstants.input14, ShooterParametersConstants.flywheelSpeed14);
        flywheelInterpolator.add(ShooterParametersConstants.input15, ShooterParametersConstants.flywheelSpeed15);
        flywheelInterpolator.add(ShooterParametersConstants.input16, ShooterParametersConstants.flywheelSpeed16);
        flywheelInterpolator.add(ShooterParametersConstants.input17, ShooterParametersConstants.flywheelSpeed17);
        flywheelInterpolator.add(ShooterParametersConstants.input18, ShooterParametersConstants.flywheelSpeed18);
        flywheelInterpolator.add(ShooterParametersConstants.input19, ShooterParametersConstants.flywheelSpeed19);
        flywheelInterpolator.add(ShooterParametersConstants.input20, ShooterParametersConstants.flywheelSpeed20);

        hoodInterpolator.add(ShooterParametersConstants.input1, ShooterParametersConstants.hoodAngle1);
        hoodInterpolator.add(ShooterParametersConstants.input2, ShooterParametersConstants.hoodAngle2);
        hoodInterpolator.add(ShooterParametersConstants.input3, ShooterParametersConstants.hoodAngle3);
        hoodInterpolator.add(ShooterParametersConstants.input4, ShooterParametersConstants.hoodAngle4);
        hoodInterpolator.add(ShooterParametersConstants.input5, ShooterParametersConstants.hoodAngle5);
        hoodInterpolator.add(ShooterParametersConstants.input6, ShooterParametersConstants.hoodAngle6);
        hoodInterpolator.add(ShooterParametersConstants.input7, ShooterParametersConstants.hoodAngle7);
        hoodInterpolator.add(ShooterParametersConstants.input8, ShooterParametersConstants.hoodAngle8);
        hoodInterpolator.add(ShooterParametersConstants.input9, ShooterParametersConstants.hoodAngle9);
        hoodInterpolator.add(ShooterParametersConstants.input10, ShooterParametersConstants.hoodAngle10);
        hoodInterpolator.add(ShooterParametersConstants.input11, ShooterParametersConstants.hoodAngle11);
        hoodInterpolator.add(ShooterParametersConstants.input12, ShooterParametersConstants.hoodAngle12);
        hoodInterpolator.add(ShooterParametersConstants.input13, ShooterParametersConstants.hoodAngle13);
        hoodInterpolator.add(ShooterParametersConstants.input14, ShooterParametersConstants.hoodAngle14);
        hoodInterpolator.add(ShooterParametersConstants.input15, ShooterParametersConstants.hoodAngle15);
        hoodInterpolator.add(ShooterParametersConstants.input16, ShooterParametersConstants.hoodAngle16);
        hoodInterpolator.add(ShooterParametersConstants.input17, ShooterParametersConstants.hoodAngle17);
        hoodInterpolator.add(ShooterParametersConstants.input18, ShooterParametersConstants.hoodAngle18);
        hoodInterpolator.add(ShooterParametersConstants.input19, ShooterParametersConstants.hoodAngle19);
        hoodInterpolator.add(ShooterParametersConstants.input20, ShooterParametersConstants.hoodAngle20);

        flywheelInterpolator.createLUT();
        hoodInterpolator.createLUT();
    }

    public void update()
    {
        xRobot = getRobotPos.get().xMeters;
        yRobot = getRobotPos.get().yMeters;
        headingRobot = getRobotPos.get().headingRadians;

        calculateRobotspeed();

        distanceToTarget = Math.sqrt(
                Math.pow(targetPos.xMeters - getRobotPos.get().xMeters, 2)
                        + Math.pow(targetPos.yMeters - getRobotPos.get().yMeters, 2)
        );

        shootParam.flywheelSpeed = flywheelInterpolator.get(distanceToTarget);
        shootParam.hoodAngle = hoodInterpolator.get(distanceToTarget);


        double xTargetRobot = Math.cos(headingRobot - TurretConstants.offsetHeading) * (targetPos.xMeters - xRobot)
                + Math.sin(headingRobot - TurretConstants.offsetHeading) * (targetPos.yMeters - yRobot);

        double yTargetRobot = -Math.sin(headingRobot - TurretConstants.offsetHeading) * (targetPos.xMeters - xRobot)
                + Math.cos(headingRobot - TurretConstants.offsetHeading) * (targetPos.yMeters - yRobot);

        shootParam.targetTurretAngle = Math.atan2(yTargetRobot, xTargetRobot);
    }

    private void calculateRobotspeed()
    {
        //for future
    }


    //private double getTurretTargetAngle() {
    //    double xRobot = getRobotPos.get().xMeters;
    //    double yRobot = getRobotPos.get().yMeters;
    //    double headingRobot = getRobotPos.get().headingRadians;
//
    //    double xTargetRobot = Math.cos(headingRobot - TurretConstants.offsetHeading) * (targetPos.xMeters - xRobot)
    //            + Math.sin(headingRobot - TurretConstants.offsetHeading) * (targetPos.yMeters - yRobot);
//
    //    double yTargetRobot = -Math.sin(headingRobot - TurretConstants.offsetHeading) * (targetPos.xMeters - xRobot)
    //            + Math.cos(headingRobot - TurretConstants.offsetHeading) * (targetPos.yMeters - yRobot);
//
    //    return Math.atan2(yTargetRobot, xTargetRobot);
    //}
}
