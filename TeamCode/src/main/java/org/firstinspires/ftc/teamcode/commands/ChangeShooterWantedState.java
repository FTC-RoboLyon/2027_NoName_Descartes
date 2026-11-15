package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
public class ChangeShooterWantedState extends CommandBase {
    private ShooterSubsystem shooter;
    private ShooterSubsystem.WantedState wantedState;
    public ChangeShooterWantedState (ShooterSubsystem shooter, ShooterSubsystem.WantedState wantedState)
    {
        this.shooter = shooter;
        this.wantedState = wantedState;
    }

    @Override
    public void initialize() {
        shooter.setWantedState(wantedState);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
