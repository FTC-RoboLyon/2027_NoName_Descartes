package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.control.PidRBL;

import java.util.List;

@TeleOp
@Config
public class ShooterCompressionTest extends OpMode {
    private DcMotorEx shooterMotor;

    private boolean isEnabled = false;

    private double motorPower = 0.0;

    private final double GEAR_RATIO = 1.0;

    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public static double KV = 0.0;
    public static double KS = 0.0;

    public static double speedTarget = 5000; //in rpm

    private PidRBL PidController;

    private FeedForwardModel FeedForwardController;

    private List<LynxModule> sensors = hardwareMap.getAll(LynxModule.class);

    private VoltageSensor voltageSensor;


    @Override
    public void init()
    {
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        voltageSensor = hardwareMap.get(VoltageSensor.class,("Control Hub"));

        PidController = new PidRBL(KP,KI,KD);
        FeedForwardController = new FeedForwardModel(KS,KV,0.0);

        for (LynxModule module : sensors)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop()
    {
        for (LynxModule module : sensors)
        {
            module.clearBulkCache();
        }

        double motorVelocity = shooterMotor.getVelocity()/28*60;
        double flywheelVelocity = motorVelocity/GEAR_RATIO;
        double batteryVoltage = voltageSensor.getVoltage();


        motorPower = 0.0;

        if (isEnabled)
        {
            double FFOutput = FeedForwardController.calculate(speedTarget);
            double PIDCorrection = PidController.calculate(speedTarget, motorVelocity);
            motorPower = FFOutput + PIDCorrection;
        }

        if (gamepad1.rightBumperWasPressed())
        {
            isEnabled = !isEnabled;
        }

        if (gamepad1.leftBumperWasPressed())
        {
            PidController.setGains(KP,KI,KD);
            FeedForwardController.setGains(KS,KV,0.0);
        }

        motorPower = motorPower*batteryVoltage/11.5;
        shooterMotor.setPower(motorPower);

        telemetry.addData("motor speed (rpm)", motorVelocity);
        telemetry.addData("flywheel speed (rpm)", flywheelVelocity);
        telemetry.update();
    }
}
