package org.firstinspires.ftc.teamcode.tests.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryWithPinpoint;

@TeleOp
public class DrivetrainTest extends OpMode {
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;

    private double leftBackPower, rightBackPower, leftFrontPower, rightFrontPower;

    private boolean fieldOriented = false;

    private OdometryWithPinpoint odo;

    private Pose2d robotPos;

    @Override
    public void init()
    {
        leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBackMotor = hardwareMap.get(DcMotor.class,"leftBackMotor");
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBackMotor = hardwareMap.get(DcMotor.class,"rightBackMotor");
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //OdometryWithPinpoint.OdometryPod XPod = new OdometryWithPinpoint.OdometryPod(),
        //OdometryWithPinpoint.OdometryPod YPod = new OdometryWithPinpoint.OdometryPod(),
        //odo = new OdometryWithPinpoint()
    }

    @Override
    public void loop()
    {
        robotPos = odo.Update();

        double XPower = -gamepad1.left_stick_y;
        double YPower = -gamepad1.left_stick_x;
        double RotationPower = -gamepad1.right_stick_x;

        if (fieldOriented)
        {
            double temp = XPower;
            XPower = Math.cos(robotPos.headingRadians)*XPower-Math.sin(robotPos.headingRadians)*YPower;
            YPower = Math.sin(robotPos.headingRadians)*temp-Math.cos(robotPos.headingRadians)*YPower;
        }

        leftBackPower = XPower + YPower - RotationPower;
        rightBackPower = XPower - YPower + RotationPower;
        leftFrontPower = XPower - YPower - RotationPower;
        rightFrontPower = XPower + YPower + RotationPower;

        double MaxPower = Math.abs(XPower)+Math.abs(YPower)+Math.abs(RotationPower);

        if (MaxPower>1)
        {
            leftBackPower = leftBackPower / MaxPower;
            rightBackPower = rightBackPower / MaxPower;
            leftFrontPower = leftFrontPower / MaxPower;
            rightFrontPower = rightFrontPower / MaxPower;
        }

        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);

        telemetry.addData("Robot Pos", robotPos.toArray());
        telemetry.update();

        if (gamepad1.aWasPressed() || gamepad1.crossWasPressed())
        {
            fieldOriented = !fieldOriented;
        }
    }
}
