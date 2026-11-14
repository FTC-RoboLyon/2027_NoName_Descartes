package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterPollen.PollenShooterSubsystem;
public class ChangeShooterWantedState extends CommandBase {
    private PollenShooterSubsystem shooter;
    private PollenShooterSubsystem.WantedState wantedState;
    public ChangeShooterWantedState (PollenShooterSubsystem shooter, PollenShooterSubsystem.WantedState wantedState)
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
