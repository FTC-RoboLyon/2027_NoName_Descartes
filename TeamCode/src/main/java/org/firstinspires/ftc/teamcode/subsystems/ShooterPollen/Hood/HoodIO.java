package org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Hood;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.Flywheel.FlywheelConstants;

public class HoodIO {
    private Servo hoodServo;

    private double hoodAngle = 0.0;
    private double hoodPosTick = 0.0;
    private double hoodServoPower = 0.0;
    private boolean isOutOfRange = false;

    private double voltage = 0.0;

    public double getHoodAngle(){return hoodAngle;}

    public HoodIO (HardwareMap hmap)
    {
        hoodServo = hmap.get(Servo.class, HoodConstants.HOOD_SERVO_NAME);

        //hoodEncodeur = hmap.get(Motor.Encoder.class, HoodConstants.HOOD_ENCODEUR_NAME);

        hoodServo.setPosition(HoodConstants.HOOD_DEFAULT_POS);
    }

    public void update()
    {
        
    }

    public void setPos(double position)
    {
        position = UtilsRBL.clamp(position, HoodConstants.HOOD_ANGLE_MIN, HoodConstants.HOOD_ANGLE_MAX);
        hoodServo.setPosition(-1/(HoodConstants.HOOD_ANGLE_MIN - HoodConstants.HOOD_ANGLE_MAX) * (position - HoodConstants.HOOD_ANGLE_MIN) ); 
        
        /* 
        Servo Position = 
        s1 − s2
        -------- (α − a1) + s1
        a1 − a2
        where:
        α = desired angle
        a1 = min angle = 0.5235987755982988 rad
        a2 = max angle = 0.75049157835756 rad
        s1 = min servo position = 0
        s2 = max servo position = 1
         
        */
    }
}
