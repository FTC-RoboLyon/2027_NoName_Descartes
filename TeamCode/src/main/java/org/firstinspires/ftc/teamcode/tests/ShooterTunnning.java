package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Hood.HoodConstants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Hood.HoodIO;

@Config
@TeleOp
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



    //HoodIO hood;

    private PidRBL hoodPID = new PidRBL(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);

    public double hoodTargetAngle = 1000; //in RPM

    private boolean isAtTargetAngle = false;
    public  double HOOD_KP = 0.0; //from Radiants to duty cycle
    public  double HOOD_KI = 0.0; //from Radiants to duty cycle
    public  double HOOD_KD = 0.0; //from Radiants to duty cycle

    public  double HOOD_TOLERANCE = 0.0; //in Radiants

    private MultipleTelemetry m_Telemetry;

    @Override
    public void init() {
        m_Telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = new FlywheelIO(hardwareMap, m_Telemetry);

        flywheelPID.setGains(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);
        flywheelFeedForward.setGains(FLYWHEEL_MOTOR_KS, FLYWHEEL_MOTOR_KV, 0.0);

        flywheelPID.setOutputLimits(-1.0, 1.0);
        flywheelFeedForward.setOutputLimits(-1.0, 1.0);

        //hood = new HoodIO(hardwareMap);
        hoodPID.setGains(HOOD_KP, HOOD_KI, HOOD_KD);
        hoodPID.setOutputLimits(-1.0, 1.0);

    }

    @Override
    public void loop() {
        flywheel.update();
        double flywheelSpeed = flywheel.getFlywheelVelocity();

        double voltage = VoltageCompensator.getVoltage();

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
        //hood.setVoltage(hoodPID.calculate(hoodTargetAngle, hood.getHoodAngle()) * (HoodConstants.HOOD_NOMINAL_VOLTAGE/voltage));

        isAtTargetSpeed = UtilsRBL.isInRange(flywheelSpeed, flywheelTargetSpeed, FLYWHEEL_TOLERANCE);
        //isAtTargetAngle = UtilsRBL.isInRange(hood.getHoodAngle(), hoodTargetAngle, HOOD_TOLERANCE);

        m_Telemetry.addData(" MY flywheel velocity in RPM", flywheelSpeed);
        m_Telemetry.addLine();

        m_Telemetry.addData("flywheel target speed", flywheelTargetSpeed);
        m_Telemetry.addLine();

        //telemetry.addData("hood Angle in Rad", hood.getHoodAngle());
        //telemetry.addLine();

        //telemetry.addData("hood target Angle in Rad", hoodTargetAngle);
        //telemetry.addLine();
//
        //telemetry.addData("isAtTargetAngle", isAtTargetAngle);
        //telemetry.addLine();

        m_Telemetry.addData("isAtTargetSpeed", isAtTargetSpeed);
        m_Telemetry.update();
//
        //m_Telemetry.addData("flywheel PID state", flywheelPID.getState());
        //m_Telemetry.addLine();
//
        //m_Telemetry.addData("flywheel feedForward state", flywheelFeedForward.getState());
        //telemetry.addLine();
//
        //m_Telemetry.addData("hood PID state", hoodPID.getState());
        //m_Telemetry.addLine();

        m_Telemetry.update();


    }
}
