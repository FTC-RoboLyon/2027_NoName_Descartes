package org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelIO {

    private Telemetry telemetry;
    private DcMotorEx flywheelMotor;
    private double flywheelMotorPower = 0.0;
    private double flywheelVelocity = 0.0; //In RPM

    private double lastPos = 0.0;
    private double lastTime = 0.0;
    private double lastSpeed = 0.0;
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
        telemetry.addData("consigne", flywheelMotor.getPower());
        voltage = VoltageCompensator.getVoltage();
        //telemetry.addData("voltage", voltage);
    }

    public void applyPower(double power)
    {
        flywheelMotor.setPower(UtilsRBL.clamp(power * (FlywheelConstants.FLYWHEEL_NOMINAL_VOLTAGE/voltage), -1.0, 1.0));
    }

    public void reset(){
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        firstIteration = true;
    }

    private void calculateFlywheelVelocity(){
        double currentPos = flywheelMotor.getCurrentPosition();
        double dPos = currentPos - lastPos; // in ticks

        double actualTime = System.nanoTime() / 1.0e9; // in seconds
        double dTime = actualTime - lastTime;

        if(firstIteration)
        {
            lastPos = currentPos;
            lastTime = actualTime;
            firstIteration = false;
        }

        double tickPerSeconds = dPos / dTime; // in ticks per second
        flywheelVelocity = tickPerSeconds / FlywheelConstants.FLYWHEEL_TICKS_PER_ROTATION * 60; //In RPM
        flywheelVelocity = FlywheelConstants.FLYWHEEL_VELOCITY_FILTER_ALPHA * flywheelVelocity + (1 - FlywheelConstants.FLYWHEEL_VELOCITY_FILTER_ALPHA) * lastSpeed;

        lastPos = currentPos;
        lastTime = actualTime;
        lastSpeed = flywheelVelocity;

    }
}
