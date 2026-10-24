package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;

public class TurretIO {
    private DcMotor turretMotor;
    private AnalogInput hallSensor;

    private double headingTicks = 0.0;
    private double headingRadiant = 0.0;
    private boolean hallSensorTrigered = false;
    private double voltage = 0.0;

    public double getHeadingRadiant(){return headingRadiant;};
    public double getHeadingTicks(){return headingTicks;};
    public double getTurretVoltage(){return turretMotor.getPower();}
    public boolean isHallSensorTrigered(){return hallSensorTrigered;}

    public TurretIO (HardwareMap hmap){

        turretMotor = hmap.get(DcMotor.class, TurretConstants.TURRET_NAME);
        hallSensor = hmap.get(AnalogInput.class, TurretConstants.HALL_SENSOR_NAME);

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void update(){
        headingTicks = -turretMotor.getCurrentPosition();
        headingRadiant = headingTicks / TurretConstants.TICKS_PER_RADIANT;

        //  hallSensorTrigered = UtilsRBL.isInRange(hallSensor.getVoltage(), TurretConstants.hallSensorMinWhenMagnet, TurretConstants.hallSensorTolerance);

        voltage = VoltageCompensator.getVoltage();
    }
    public void applyPower(double power){
        turretMotor.setPower(power * (TurretConstants.TURRET_NOMINAL_VOLTAGE / voltage));
    }

    public void resetMotor(){
        turretMotor.setPower(0.0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
