package org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Hood;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Flywheel.FlywheelConstants;

public class HoodIO {
    private CRServo hoodServo;
    private Motor.Encoder hoodEncodeur;

    private double hoodAngle = 0.0;
    private double hoodPosTick = 0.0;
    private double hoodServoPower = 0.0;
    private boolean isOutOfRange = false;

    private double voltage = 0.0;

    public double getHoodAngle(){return hoodAngle;}

    public HoodIO (HardwareMap hmap)
    {
        hoodServo = hmap.get(CRServo.class, HoodConstants.HOOD_SERVO_NAME);

        hoodEncodeur = hmap.get(Motor.Encoder.class, HoodConstants.HOOD_ENCODEUR_NAME);

        hoodServo.setDirection(DcMotorSimple.Direction.FORWARD); //not sure
        hoodServo.setPower(0.0);
    }

    public void update()
    {
        hoodPosTick = hoodEncodeur.getPosition();
        hoodAngle = hoodPosTick/HoodConstants.HOOD_TICK_PER_RADIANT;
        isOutOfRange = hoodAngle > HoodConstants.HOOD_ANGLE_MAX || hoodAngle < HoodConstants.HOOD_ANGLE_MIN;

        if (isOutOfRange)
            hoodServo.setPower(0.0);
        voltage = VoltageCompensator.getVoltage();
    }

    public void setVoltage(double power)
    {
        if (!isOutOfRange)
            hoodServo.setPower(power * (HoodConstants.HOOD_NOMINAL_VOLTAGE/voltage));
    }
}
