package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

public class ChangeShooterControlMode extends CommandBase {

    private ShooterSubsystem subsystem;
    private ControlMode mode;
    public ChangeShooterControlMode(ShooterSubsystem subsystem, ControlMode mode)
    {
        this.subsystem = subsystem;
        this.mode = mode;
    }
    @Override
    public void initialize()
    {
        subsystem.setShooterControlMode(mode);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
