package org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelIO {

    private Telemetry telemetry;
    private DcMotorEx flywheelMotor;
    private double flywheelMotorPower = 0.0;
    private double flywheelVelocity = 0.0; //In RPM

    private double lastPos = 0.0;
    private double lastTime = 0.0;
    private boolean firstIteration = true;

    private double voltage = 0.0;

    public double getFlywheelMotorPower() {return flywheelMotorPower;}
    public double getFlywheelVelocity() {return flywheelVelocity;}

    public FlywheelIO (HardwareMap hmap)
    {
        flywheelMotor = hmap.get(DcMotorEx.class, FlywheelConstants.FLYWHEEL_MOTOR_NAME);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setPower(0.0);

        reset();
    }

    public FlywheelIO (HardwareMap hmap, Telemetry telemetry)
    {
        this(hmap);

        this.telemetry = telemetry;
    }

    public void update(){
        calculateFlywheelVelocity();

        voltage = VoltageCompensator.getVoltage();
    }

    public void applyPower(double power)
    {
        flywheelMotor.setPower(power * (FlywheelConstants.FLYWHEEL_NOMINAL_VOLTAGE/voltage));
    }

    public void reset(){
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        firstIteration = true;
    }

    private void calculateFlywheelVelocity(){
        double currentPos = flywheelMotor.getCurrentPosition();
        double dPos = currentPos - lastPos; // in ticks

        double actualTime = (double) System.nanoTime() / 1_000_000_000; // in seconds
        double dTime = actualTime - lastTime;

        if(firstIteration)
        {
            lastPos = currentPos;
            lastTime = actualTime;
            firstIteration = false;
        }

        double tickPerSeconds = dPos / dTime; // in ticks per second
        flywheelVelocity = tickPerSeconds / FlywheelConstants.FLYWHEEL_TICKS_PER_ROTATION * 60; //In RPM

        lastPos = currentPos;
        lastTime = actualTime;

        telemetry.addData("current flywheel ticks pos", currentPos);
        telemetry.addData("flywheel delta ticks pos", dPos);

        telemetry.addData("current time", actualTime);
        telemetry.addData("current delta time", dTime);

        telemetry.addData("COM QUALCORE flywheel velocity", flywheelMotor.getVelocity()/ FlywheelConstants.FLYWHEEL_TICKS_PER_ROTATION * 60);
    }
}
