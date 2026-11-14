package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.PollenShooterSubsystem;

public class ChangeFlywheelControlMode extends CommandBase {
    private PollenShooterSubsystem shooter;
    private ControlMode controlMode;
    public ChangeFlywheelControlMode (PollenShooterSubsystem shooter, ControlMode controlMode)
    {
        this.shooter = shooter;
        this.controlMode = controlMode;
    }

    @Override
    public void initialize() {
        shooter.setFlywheelControlMode(controlMode);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
