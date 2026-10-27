package org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;

public class HoodIO {
    private Servo hood;
    private Servo pollenisator;
    private Servo nectarisator;
    private double voltage = 0.0;

    public HoodIO (HardwareMap hmap){
        hood = hmap.get(Servo.class, HoodConstants.HOOD_NAME);
        pollenisator = hmap.get(Servo.class, HoodConstants.POLLENISATOR_NAME);
        nectarisator = hmap.get(Servo.class, HoodConstants.NECTARISATOR_NAME);
    }
    public void update()
    {
        voltage = VoltageCompensator.getVoltage();
    }

    public void packEverything()
    {
        hood.setPosition(HoodConstants.HOOD_DEFAULT_POS);
        pollenisator.setPosition(HoodConstants.POLLENISATOR_RESTING_POS);
        nectarisator.setPosition(HoodConstants.NECTARISATOR_RESTING_POS);
    }
    public void setHoodPosition(double position)
    {
        hood.setPosition(position);
    }
    public void setPollenisatorPosition(double position)
    {
        pollenisator.setPosition(position);
    }
    public void setNectarisatorPosition(double position)
    {
        nectarisator.setPosition(position);
    }
}
