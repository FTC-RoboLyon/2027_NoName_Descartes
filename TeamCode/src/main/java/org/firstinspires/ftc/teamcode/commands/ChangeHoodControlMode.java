package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

public class ChangeHoodControlMode extends CommandBase {
    private ShooterSubsystem shooter;
    private ControlMode controlMode;
    public ChangeHoodControlMode (ShooterSubsystem shooter, ControlMode controlMode)
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

