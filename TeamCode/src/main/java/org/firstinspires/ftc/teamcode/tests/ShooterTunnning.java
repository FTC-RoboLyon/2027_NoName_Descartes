package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodIO;

@Config
@TeleOp
public class ShooterTunnning extends OpMode {

    FlywheelIO flywheel;
    private PidRBL flywheelPID = new PidRBL(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);
    private FeedForwardModel flywheelFeedForward = new FeedForwardModel(FLYWHEEL_MOTOR_KS, FLYWHEEL_MOTOR_KV, 0.0);

    public static double flywheelTargetSpeed = 1000; //in RPM
    public static double flywheelDutycycle = 0.0;
    private boolean isAtTargetSpeed = false;

    public static double FLYWHEEL_MOTOR_KP = 0.0030;  //from RPM to duty cycle
    public static double FLYWHEEL_MOTOR_KI = 0.0007; //from RPM to duty cycle
    public static double FLYWHEEL_MOTOR_KD = 0.00075; //from RPM to duty cycle

    public static double FLYWHEEL_MOTOR_KS = 0.335; //from RPM to duty cycle
    public static double FLYWHEEL_MOTOR_KV = (double) 1/2255 ; //from RPM to duty cycle

    public static double FLYWHEEL_TOLERANCE = 0.0; //in RPM



    HoodIO hood;

    public static double hoodTargetAngle = 0.1; 

    private boolean isAtTargetAngle = false;

    public static double HOOD_TOLERANCE = 0.0; //in Radiants

    private MultipleTelemetry m_Telemetry;
    private VoltageCompensator voltageCompensator;

    @Override
    public void init() {
        m_Telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = new FlywheelIO(hardwareMap, m_Telemetry);

        flywheelPID.setGains(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);
        flywheelFeedForward.setGains(FLYWHEEL_MOTOR_KS, FLYWHEEL_MOTOR_KV, 0.0, null);

        flywheelPID.setOutputLimits(-1.0, 1.0);
        flywheelFeedForward.setOutputLimits(-1.0, 1.0);

        //hood = new HoodIO(hardwareMap);
        
        voltageCompensator = new VoltageCompensator(hardwareMap);
    }

    @Override
    public void loop() {
        flywheel.update();
        double flywheelSpeed = flywheel.getFlywheelVelocity();

        voltageCompensator.update();
        double voltage = VoltageCompensator.getVoltage();


        flywheelPID.setGains(FLYWHEEL_MOTOR_KP, FLYWHEEL_MOTOR_KI, FLYWHEEL_MOTOR_KD);
        flywheelFeedForward.setGains(FLYWHEEL_MOTOR_KS, FLYWHEEL_MOTOR_KV, 0.0, null);

        double pidOutput = flywheelPID.calculate(flywheelTargetSpeed, flywheelSpeed);
        flywheel.applyPower(
                flywheelFeedForward.calculate(flywheelTargetSpeed)
                +pidOutput
                );

        m_Telemetry.addData( "feedForwardOutput", flywheelFeedForward.calculate(flywheelTargetSpeed));
        m_Telemetry.addData( "fPIDOutput", pidOutput);
        //hood.setPos(hoodTargetAngle);

        isAtTargetSpeed = UtilsRBL.isInRange(flywheelSpeed, flywheelTargetSpeed, FLYWHEEL_TOLERANCE);
        //isAtTargetAngle = UtilsRBL.isInRange(hood.getHoodAngle(), hoodTargetAngle, HOOD_TOLERANCE);

        m_Telemetry.addData(" flywheel velocity in RPM", flywheelSpeed);
        m_Telemetry.addLine();

        m_Telemetry.addData("flywheel target speed", flywheelTargetSpeed);
        m_Telemetry.addLine();

        //telemetry.addData("hood Angle in Rad", hood.getHoodAngle());
        //telemetry.addLine();

        //telemetry.addData("hood target Angle in Rad", hoodTargetAngle);
        //telemetry.addLine();

        //telemetry.addData("isAtTargetAngle", isAtTargetAngle);
        //telemetry.addLine();

        m_Telemetry.addData("isAtTargetSpeed", isAtTargetSpeed);
        m_Telemetry.update();

        //m_Telemetry.addData("flywheel PID state", flywheelPID.getState());
        //m_Telemetry.addLine();

        //m_Telemetry.addData("flywheel feedForward state", flywheelFeedForward.getState());
        //telemetry.addLine();

        //m_Telemetry.addData("hood PID state", hoodPID.getState());
        //m_Telemetry.addLine();

        m_Telemetry.update();


    }
}
