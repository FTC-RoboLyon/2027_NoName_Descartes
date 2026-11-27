package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.superstructure.Superstructure;

public class StopSuperFrontIntakeCmd extends CommandBase{

    private Superstructure superstructure;

    public StopSuperFrontIntakeCmd(Superstructure superstructure)
    {
        this.superstructure = superstructure;
    }

    @Override
    public void initialize()
    {
        // switch(superstructure.GetSystemSuperState())
        // {
            // case FRONT_INTAKING:
                // superstructure.SetWantedSuperState(WantedSuperState.STAND_BY);
                // break;
// 
            // case DOUBLE_INTAKING:
                // superstructure.SetWantedSuperState(WantedSuperState.BACK_INTAKING);
// 
            // case SHOOTING_WHILE:
                // superstructure.SetWantedSuperState(WantedSuperState.BACK_INTAKING);
// 
            // default:
                // break;      
        // }
    }

    @Override 
    public boolean isFinished()
    {
        return true;
    }
}