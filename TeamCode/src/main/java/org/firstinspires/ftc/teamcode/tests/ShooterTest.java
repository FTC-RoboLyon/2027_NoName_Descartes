package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ShooterTest extends OpMode {
    DcMotor shooterMotor;

    private boolean inClosedLoop = false;

    private double motorPower = 0.0;
    private double manual_coef = 1.0;

    private Servo jspCommentTappeler;

    private double pollenPos = 0.5;
    private double nectarPos = 1.0;

    @Override
    public void init()
    {
        shooterMotor = hardwareMap.get(DcMotor.class,"shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        jspCommentTappeler = hardwareMap.get(Servo.class, "trucDuShooter");
    }

    @Override
    public void loop()
    {
        motorPower = 0.0;

        if (inClosedLoop)
        {
            //TODO:program closed loop
        }
        else
        {
            motorPower = (gamepad1.right_trigger - gamepad1.left_trigger)*manual_coef;
        }

        if(gamepad1.b)
        {
            jspCommentTappeler.setPosition(nectarPos);
        }

        if (gamepad1.a)
        {
            jspCommentTappeler.setPosition(pollenPos);
        }

        shooterMotor.setPower(motorPower);
    }
}
