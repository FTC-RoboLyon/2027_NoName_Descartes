package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp
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
        private ArrayList<BallType> balls = new ArrayList();
    }


    private DcMotor shooterMotor;
    private double shooterPower = 0.0;


    private DcMotor feederMotor;
    private double feederPower = 0.0;


    private Servo jspCommentTappeler;
    private double pollenPos = 0.5;
    private double nectarPos = 1.0;


    private List<LynxModule> sensors = hardwareMap.getAll(LynxModule.class);
    private AnalogInput LowIR, HighIR, LimitSwitch;
    private Inputs inputs = new Inputs();
    private double IrTrueVoltage = 0.0, SwitchTrueVoltage = 0.0;//TUNEME


    @Override
    public void init()
    {
        shooterMotor = hardwareMap.get(DcMotor.class,"shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        feederMotor = hardwareMap.get(DcMotor.class,"feederMotor");
        feederMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setDirection(DcMotor.Direction.FORWARD);

        LowIR = hardwareMap.get(AnalogInput.class, "LowIR");
        HighIR = hardwareMap.get(AnalogInput.class, "HighIR");
        LimitSwitch = hardwareMap.get(AnalogInput.class, "LimitSwitch");

        jspCommentTappeler = hardwareMap.get(Servo.class, "trucDuShooter");

        for(LynxModule module : sensors)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop()
    {
        UpdateSensors();

        shooterPower = gamepad1.right_trigger - gamepad1.left_trigger;
        feederPower = shooterPower*0.5;

        if(!inputs.balls.isEmpty())
        {
            if(inputs.balls.get(0) == BallType.POLLEN)
            {
                jspCommentTappeler.setPosition(pollenPos);
            }
            else
            {
                jspCommentTappeler.setPosition(nectarPos);
            }
        }

        shooterMotor.setPower(shooterPower);
        feederMotor.setPower(feederPower);

        telemetry.addData("Balls", inputs.balls.toString());
        telemetry.addData("LowIRBreaker", LowIR.getVoltage());
        telemetry.addData("HighIRBreaker", HighIR.getVoltage());
        telemetry.addData("LimitSwitch", LimitSwitch.getVoltage());
        telemetry.update();
    }

    public void UpdateSensors()
    {
        for(LynxModule module : sensors)
        {
            module.clearBulkCache();
        }

        inputs.HighIRWasTriggered = inputs.HighIRTriggered;
        inputs.HighIRTriggered = HighIR.getVoltage() == IrTrueVoltage;

        inputs.LowIRWasTriggered = inputs.LowIRTriggered;
        inputs.LowIRTriggered = LowIR.getVoltage() == IrTrueVoltage;

        inputs.LimitSwitchWasTriggered = inputs.LimitSwitchTriggered;
        inputs.LimitSwitchTriggered = LimitSwitch.getVoltage() == SwitchTrueVoltage;

        if(!inputs.LowIRTriggered && inputs.LowIRWasTriggered)
        {
            if(inputs.LimitSwitchTriggered)
            {
                inputs.balls.add(BallType.NECTAR);
            }
            else
            {
                inputs.balls.add(BallType.POLLEN);
            }
        }

        if(!inputs.HighIRTriggered && inputs.HighIRWasTriggered)
        {
            inputs.balls.remove(0);
        }
    }
}
