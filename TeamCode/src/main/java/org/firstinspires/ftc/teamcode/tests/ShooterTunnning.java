package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Hood.HoodIO;

@Config
public class ShooterTunnning extends OpMode {

    FlywheelIO flywheel;
    private PidRBL flywheelPID = new PidRBL(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);
    private FeedForwardModel flywheelFeedForward = new FeedForwardModel(FLYWHEEL_MOTOR_KS, FLYWHEEL_MOTOR_KV, 0.0);

    public static double flywheelTargetSpeed = 1000; //in RPM
    private boolean isAtTargetSpeed = false;

    public static double FLYWHEEL_MOTOR_KP = 0.0;  //from RPM to duty cycle
    public static double FLYWHEEL_MOTOR_KI = 0.0; //from RPM to duty cycle
    public static double FLYWHEEL_MOTOR_KD = 0.0; //from RPM to duty cycle

    public static double FLYWHEEL_MOTOR_KS = 0.0; //from RPM to duty cycle
    public static double FLYWHEEL_MOTOR_KV = 0.0; //from RPM to duty cycle

    public static double FLYWHEEL_TOLERANCE = 0.0; //in RPM



    HoodIO hood;

    private PidRBL hoodPID = new PidRBL(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);

    public static double hoodTargetAngle = 1000; //in RPM

    private boolean isAtTargetAngle = false;
    public static double HOOD_KP = 0.0; //from Radiants to duty cycle
    public static double HOOD_KI = 0.0; //from Radiants to duty cycle
    public static double HOOD_KD = 0.0; //from Radiants to duty cycle

    public static double HOOD_TOLERANCE = 0.0; //in Radiants

    @Override
    public void init() {
        flywheel = new FlywheelIO(hardwareMap);

        flywheelPID.setGains(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);
        flywheelFeedForward.setGains(FLYWHEEL_MOTOR_KS, FLYWHEEL_MOTOR_KV, 0.0);

        flywheelPID.setOutputLimits(-1.0, 1.0);
        flywheelFeedForward.setOutputLimits(-1.0, 1.0);

        hood = new HoodIO(hardwareMap);
        hoodPID.setGains(HOOD_KP, HOOD_KI, HOOD_KD);
        hoodPID.setOutputLimits(-1.0, 1.0);
    }

    @Override
    public void loop() {
        flywheel.update();
        double flywheelSpeed = flywheel.getFlywheelVelocity();

        if (gamepad1.aWasPressed())
        {
            flywheelPID.setGains(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);
            flywheelFeedForward.setGains(FLYWHEEL_MOTOR_KS, FLYWHEEL_MOTOR_KV, 0.0);
        }
        if (gamepad1.bWasPressed())
            hoodPID.setGains(HOOD_KP, HOOD_KI, HOOD_KD);

        flywheel.applyPower(
                flywheelFeedForward.calculate(flywheelSpeed)
                + flywheelPID.calculate(flywheelTargetSpeed, flywheelSpeed)
        );
        hood.setVoltage(hoodPID.calculate(hoodTargetAngle, hood.getHoodAngle()));

        isAtTargetSpeed = UtilsRBL.isInRange(flywheelSpeed, flywheelTargetSpeed, FLYWHEEL_TOLERANCE);
        isAtTargetAngle = UtilsRBL.isInRange(hood.getHoodAngle(), hoodTargetAngle, HOOD_TOLERANCE);

        telemetry.addData("flywheel velocity in RPM", flywheelSpeed);
        telemetry.addLine();

        telemetry.addData("flywheel target speed", flywheelTargetSpeed);
        telemetry.addLine();

        telemetry.addData("hood Angle in Rad", hood.getHoodAngle());
        telemetry.addLine();

        telemetry.addData("hood target Angle in Rad", hoodTargetAngle);
        telemetry.addLine();

        telemetry.addData("isAtTargetAngle", isAtTargetAngle);
        telemetry.addLine();

        telemetry.addData("isAtTargetSpeed", isAtTargetSpeed);
        telemetry.update();

        telemetry.addData("flywheel PID state", flywheelPID.getState());
        telemetry.addLine();

        telemetry.addData("flywheel feedForward state", flywheelFeedForward.getState());
        telemetry.addLine();

        telemetry.addData("hood PID state", hoodPID.getState());
        telemetry.addLine();

        FtcDashboard.getInstance().getTelemetry().addData("flywheel velocity in RPM", flywheelSpeed);
        FtcDashboard.getInstance().getTelemetry().addData("flywheel target speed", flywheelTargetSpeed);

        FtcDashboard.getInstance().getTelemetry().addData("hood Angle in Rad", hood.getHoodAngle());
        FtcDashboard.getInstance().getTelemetry().addData("hood target Angle in Rad", hoodTargetAngle);

        FtcDashboard.getInstance().getTelemetry().update();


    }
}
