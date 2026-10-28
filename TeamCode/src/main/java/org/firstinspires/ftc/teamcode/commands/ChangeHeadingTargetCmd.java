package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

public class ChangeHeadingTargetCmd extends CommandBase {
    DrivetrainSubsystem drivetrain;
    double value;
    boolean robotRelative;
    public ChangeHeadingTargetCmd(DrivetrainSubsystem drivetrain, double value, boolean robotRelative)
    {
        this.drivetrain = drivetrain;
        this.robotRelative = robotRelative;
        this.value = value;
    }

    @Override
    public void initialize()
    {
        if (robotRelative)
        {
            drivetrain.incrementHeadingTarget(value);
        }
        else
        {
            drivetrain.setTargetHeading(value);
        }
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
