package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;


@TeleOp
@Config
public class TestTeleopV1 extends OpMode{
    RobotContainer robotContainer;

    // --- Turret ---
    private CRServoImpl turretLeftServo, turretRightServo;
    private double servoPower;

    // --- Shooter ---
    private DcMotorEx shooterFrontMotor, shooterBackMotor;
    private double shooterPower;
    private Servo jspCommentTappeler, lePoteDeLautre; //left, right
    public static double leftPollenPos = 0.75, leftNectarPos = 0.3;
    public static double rightPollenPos = 0.25, rightNectarPos = 0.5;
    private boolean shootingPollen = true;

    @Override
    public void init()
    {
        robotContainer = new RobotContainer(hardwareMap, telemetry,
                new GamepadEx(gamepad1), new GamepadEx(gamepad2), RobotContainer.RobotMode.TELEOP_RED);

        robotContainer.init();

        initTurret();
        initShooter();
    }

    @Override
    public void start()
    {
        robotContainer.start();
    }

    @Override
    public void loop()
    {
        robotContainer.periodic();

        turret();
        shooter();

        telemetry.addData("ShooterMotorRPM", shooterFrontMotor.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop()
    {
        robotContainer.end();
    }

    private void initTurret()
    {
        turretLeftServo = hardwareMap.get(CRServoImpl.class,"turretLeftServo");
        turretLeftServo.setDirection(CRServoImpl.Direction.FORWARD);

        turretRightServo = hardwareMap.get(CRServoImpl.class,"turretRightServo");
        turretRightServo.setDirection(CRServoImpl.Direction.FORWARD);
    }

    private void initShooter()
    {
        shooterFrontMotor = hardwareMap.get(DcMotorEx.class,"shooterMotor");
        shooterFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterFrontMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterBackMotor = hardwareMap.get(DcMotorEx.class,"clode");
        shooterBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBackMotor.setDirection(DcMotor.Direction.REVERSE);

        jspCommentTappeler = hardwareMap.get(Servo.class, "trucDuShooter");
        lePoteDeLautre = hardwareMap.get(Servo.class, "machinDuShooter");
    }

    private void turret()
    {
        servoPower = -gamepad2.left_stick_x;
        turretLeftServo.setPower(servoPower);
        turretRightServo.setPower(servoPower);
    }

    private void shooter()
    {
        shooterPower = -gamepad2.right_stick_y;
        shooterBackMotor.setPower(shooterPower);
        shooterFrontMotor.setPower(shooterPower);

        if(gamepad2.triangleWasPressed())
        {
            shootingPollen = !shootingPollen;
        }

        if(shootingPollen)
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
}
