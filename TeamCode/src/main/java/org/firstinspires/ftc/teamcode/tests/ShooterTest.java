package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class ShooterTest extends OpMode {

    private enum BallType
    {
        NECTAR,
        POLLEN
    }

    class Inputs
    {
        public boolean LowIRWasTriggered = false;
        public boolean LowIRTriggered = false;
        public boolean HighIRWasTriggered = false;
        public boolean HighIRTriggered = false;
        public boolean LimitSwitchWasTriggered = false;
        public boolean LimitSwitchTriggered = false;

        public boolean LimitPressedAtThisIRTrigger = false;
        private final ArrayList<BallType> balls = new ArrayList();
    }


    private DcMotor shooterMotor;
    private double shooterPower = 0.0;


    //private DcMotor feederMotor;
    //private double feederPower = 0.0;
    private double lastTime = 0.0;

    private Servo jspCommentTappeler, lePoteDeLautre; //left, right
    private final double leftPollenPos = 0.45, leftNectarPos = 0.75;
    private final double rightPollenPos = 0.55, rightNectarPos = 0.25;
    private Servo hood;
    private double hoodPos = 0.5;
    public static double hoodHighLimit = 1.0, hoodLowLimit = 0.0;

    private List<LynxModule> sensors;
    //private DigitalChannel LowIR, HighIR;
    //private AnalogInput LimitSwitch;

    private final Inputs inputs = new Inputs();
    private final double SwitchTrueVoltage = 0.1;//TUNEME


    @Override
    public void init()
    {
        shooterMotor = hardwareMap.get(DcMotor.class,"shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        //feederMotor = hardwareMap.get(DcMotor.class,"feederMotor");
        //feederMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //feederMotor.setDirection(DcMotor.Direction.REVERSE);

        //LowIR = hardwareMap.get(DigitalChannel.class, "LowIR");
        //HighIR = hardwareMap.get(DigitalChannel.class, "HighIR");
        //LimitSwitch = hardwareMap.get(AnalogInput.class, "limitswitch_analog");
        //LowIR.setMode(DigitalChannel.Mode.INPUT);
        //HighIR.setMode(DigitalChannel.Mode.INPUT);

        jspCommentTappeler = hardwareMap.get(Servo.class, "trucDuShooter");
        lePoteDeLautre = hardwareMap.get(Servo.class, "machinDuShooter");
        hood = hardwareMap.get(Servo.class, "hood");

        sensors = hardwareMap.getAll(LynxModule.class);

        for(LynxModule module : sensors)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop()
    {

        UpdateSensors();
        shooterPower = (gamepad1.right_trigger - gamepad1.left_trigger)*0.7;
        //feederPower = gamepad1.right_trigger - gamepad1.left_trigger;

        if(gamepad1.dpad_up)
        {
            inputs.balls.clear();
            inputs.balls.add(BallType.NECTAR);
        }
        else
        {
            inputs.balls.clear();
            inputs.balls.add(BallType.POLLEN);
        }

        if(!inputs.balls.isEmpty())
        {
            if(inputs.balls.get(0) == BallType.POLLEN)
            {
                jspCommentTappeler.setPosition(leftPollenPos);
                lePoteDeLautre.setPosition(rightPollenPos);
            }
            else
            {
                jspCommentTappeler.setPosition(leftNectarPos);
                lePoteDeLautre.setPosition(rightNectarPos);
            }
        }

        if(gamepad1.triangleWasPressed())
        {
            hoodPos += 0.1;
        }
        else if (gamepad1.squareWasPressed())
        {
            hoodPos -= 0.1;
        }

        if (gamepad1.circle || hoodPos < hoodLowLimit)
        {
            hoodPos = hoodLowLimit;
        }
        else if (gamepad1.cross || hoodPos > hoodHighLimit)
        {
            hoodPos = hoodHighLimit;
        }

        shooterMotor.setPower(shooterPower);
        //feederMotor.setPower(feederPower);

        hood.setPosition(hoodPos);

        double now = System.nanoTime()*1e-9;
        double loopTime = now - lastTime;

        telemetry.addData("Balls", inputs.balls.toString());
        //telemetry.addData("LowIRBreaker", LowIR.getState());
        //telemetry.addData("HighIRBreaker", inputs.HighIRTriggered);
        //telemetry.addData("LimitSwitch", inputs.LimitSwitchTriggered);
        telemetry.addData("loopTime", loopTime);
        telemetry.update();
        lastTime = now;
    }

    public void UpdateSensors()
    {
        //for(LynxModule module : sensors)
        //{
        //    module.clearBulkCache();
        //}
//
        //inputs.HighIRWasTriggered = inputs.HighIRTriggered;
        //inputs.HighIRTriggered = HighIR.getState();
//
        //inputs.LowIRWasTriggered = inputs.LowIRTriggered;
        //inputs.LowIRTriggered = LowIR.getState();
//
        //inputs.LimitSwitchWasTriggered = inputs.LimitSwitchTriggered;
        //inputs.LimitSwitchTriggered = LimitSwitch.getVoltage() < SwitchTrueVoltage;
//
//
        //if(!inputs.LowIRTriggered && inputs.LowIRWasTriggered)
        //{
        //    if(inputs.LimitPressedAtThisIRTrigger)
        //    {
        //        inputs.balls.add(BallType.NECTAR);
        //    }
        //    else
        //    {
        //        inputs.balls.add(BallType.POLLEN);
        //    }
        //}
//
        //if(inputs.LowIRTriggered && inputs.LimitSwitchTriggered)
        //{
        //    inputs.LimitPressedAtThisIRTrigger = true;
        //}
        //else if (!inputs.LowIRTriggered)
        //{
        //    inputs.LimitPressedAtThisIRTrigger = false;
        //}
//
        //if(!inputs.HighIRTriggered && inputs.HighIRWasTriggered)
        //{
        //    if(!inputs.balls.isEmpty())
        //    {
        //        inputs.balls.remove(0);
        //    }
        //}
    }
}
