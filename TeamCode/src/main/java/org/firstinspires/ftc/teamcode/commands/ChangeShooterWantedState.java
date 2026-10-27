package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;

public class ChangeShooterWantedState extends CommandBase {
    private ShooterSubsystem subsystem;
    private ShooterSubsystem.WantedState state;

    public ChangeShooterWantedState(ShooterSubsystem subsystem, ShooterSubsystem.WantedState state)
    {
        this.subsystem = subsystem;
        this.state = state;
    }
    @Override
    public void initialize()
    {
        subsystem.setWantedState(state);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
