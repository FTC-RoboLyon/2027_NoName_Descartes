package org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;

public class FlywheelIO {
    private DcMotor flywheelMotor1;
    private DcMotor flywheelMotor2;
    private ElapsedTime velocityTimer;

    private double flywheelMotorPower = 0.0;
    private double flywheelVelocity = 0.0; //In RPM

    private double lastPos = 0.0;
    private double lastTime = 0.0;
    private boolean firstIteration = true;

    private double voltage = 0.0;

    public double getFlywheelMotorPower() {return flywheelMotorPower;}
    public double getFlywheelVelocity() {return flywheelVelocity;}

    public FlywheelIO (HardwareMap hmap){
        flywheelMotor1 = hmap.get(DcMotor.class, FlywheelConstants.FLYWHEEL_MOTOR1_NAME);
        flywheelMotor2 = hmap.get(DcMotor.class, FlywheelConstants.FLYWHEEL_MOTOR2_NAME);

        flywheelMotor1.setDirection(DcMotorSimple.Direction.FORWARD); //TUNEME
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE); //TUNEME

        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        velocityTimer = new ElapsedTime();

        flywheelMotor1.setPower(0.0);
        flywheelMotor2.setPower(0.0);
        reset();
    }

    public void reset(){
        flywheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityTimer.reset();
        velocityTimer.startTime();

        firstIteration = true;
    }

    public void update(){
        calculateFlywheelVelocity();
        voltage = VoltageCompensator.getVoltage();
    }

    public void applyPower(double power){
        flywheelMotor1.setPower(power * (FlywheelConstants.FLYWHEEL_NOMINAL_VOLTAGE/voltage));
        flywheelMotor2.setPower(power * (FlywheelConstants.FLYWHEEL_NOMINAL_VOLTAGE/voltage));
    }


    private void calculateFlywheelVelocity(){
        double currentPos = flywheelMotor1.getCurrentPosition(); //For now, because we don't have an encodeur on the flywheel, we use the first of the two motors
        double dPos = currentPos - lastPos;

        double actualTime = velocityTimer.milliseconds() / 1000;
        double dTime = actualTime - lastTime;

        if(firstIteration){
            lastPos = currentPos;
            lastTime = actualTime;
            firstIteration = false;
        }

        double tickPerSeconds = dPos / dTime;
        flywheelVelocity= tickPerSeconds / FlywheelConstants.FLYWHEEL_TICKS_PER_ROTATION * 60;

        lastPos = currentPos;
        lastTime = actualTime;
    }
}
