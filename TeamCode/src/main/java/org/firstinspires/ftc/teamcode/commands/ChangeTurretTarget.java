package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;

public class ChangeTurretTarget extends CommandBase {
    private TurretSubsystem turret;
    private TurretSubsystem.WantedState wantedState;
    public ChangeTurretTarget (TurretSubsystem turret, TurretSubsystem.WantedState wantedState)
    {
        this.turret = turret;
        this.wantedState = wantedState;
    }

    @Override
    public void initialize() {
        turret.setWantedState(wantedState);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
