package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;

public class OverrideDrivetrainInputsCmd extends CommandBase {
    DrivetrainSubsystem drivetrain;
    double fwd, strafe, rotation;
    public OverrideDrivetrainInputsCmd(DrivetrainSubsystem drivetrain,
                                       double fwd, double strafe, double rotation)
    {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        this.fwd = fwd;
        this.strafe = strafe;
        this.rotation = rotation;
    }

    @Override
    public void initialize()
    {}

    @Override
    public void execute()
    {
        if(Math.abs(UtilsRBL.wrapNegPiToPi(drivetrain.getCurrentPose().headingRadians)) > Math.PI/2.0)
        {
            strafe = -strafe;
        }
        drivetrain.OverrideInputs(fwd, strafe, rotation);
    }
}
