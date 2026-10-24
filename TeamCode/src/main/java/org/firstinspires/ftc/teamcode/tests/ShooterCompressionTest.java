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
    private DcMotorEx clode;


    private boolean isEnabled = false;

    private double motorPower = 0.0;
    public static double clodePower = 0.0;

    private final double GEAR_RATIO = 4.0/3.0;

    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public static double KV = 1.86e-4;
    public static double KS = 0.008;

    public static double speedTarget = 1000; //in rpm

    private PidRBL PidController;

    private FeedForwardModel FeedForwardController;

    private List<LynxModule> sensors;

    private VoltageSensor voltageSensor;


    @Override
    public void init()
    {
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        clode = hardwareMap.get(DcMotorEx.class,"clode");
        clode.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clode.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clode.setDirection(DcMotor.Direction.FORWARD);

        voltageSensor = hardwareMap.get(VoltageSensor.class,("Control Hub"));

        PidController = new PidRBL(KP,KI,KD);
        FeedForwardController = new FeedForwardModel(KS,KV,0.0);

        sensors = hardwareMap.getAll(LynxModule.class);

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

        double FFOutput = FeedForwardController.calculate(speedTarget);
        double PIDCorrection = PidController.calculate(speedTarget, motorVelocity);

        if (isEnabled)
        {
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

        if (gamepad1.a)
        {
            motorPower = 1.0;
        }

        motorPower = motorPower*batteryVoltage/11.5;
        shooterMotor.setPower(motorPower);

        clode.setPower(clodePower*batteryVoltage/11.5);

        telemetry.addData("motor speed (rpm)", motorVelocity);
        telemetry.addData("flywheel speed (rpm)", flywheelVelocity);
        telemetry.addData("feedforwardOutputs", FFOutput);
        telemetry.update();
    }
}
