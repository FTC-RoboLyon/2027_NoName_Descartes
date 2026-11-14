package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.PollenShooterSubsystem;

public class ChangeHoodControlMode extends CommandBase {
    private PollenShooterSubsystem shooter;
    private ControlMode controlMode;
    public ChangeHoodControlMode (PollenShooterSubsystem shooter, ControlMode controlMode)
    {
        this.shooter = shooter;
        this.controlMode = controlMode;
    }

    @Override
    public void initialize() {
        shooter.setHoodControlMode(controlMode);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}

