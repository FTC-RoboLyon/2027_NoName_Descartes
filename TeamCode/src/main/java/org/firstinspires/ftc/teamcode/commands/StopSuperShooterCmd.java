package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.superstructure.Superstructure;

public class StopSuperShooterCmd extends CommandBase{

    private Superstructure superstructure;

    public StopSuperShooterCmd(Superstructure superstructure)
    {
        this.superstructure = superstructure;
    }

    @Override
    public void initialize()
    {
        // switch(superstructure.GetSystemSuperState())
        // {
            // case SHOOTING:
            // case FEEDING:
            // case PREPARING_SHOOT:
            // case PREPARING_FEED:
                // superstructure.SetWantedSuperState(WantedSuperState.STAND_BY);
                // break;
// 
            // case SHOOTING_WHILE_FRONT_INTAKING:
            // case PREPARING_SHOOT_WHILE_FRONT_INTAKING:
            // case FEEDING_WHILE_FRONT_INTAKING:
            // case PREPARING_FEED_WHILE_FRONT_INTAKING:
                // superstructure.SetWantedSuperState(WantedSuperState.FRONT_INTAKE);
                // break;
// 
            // case SHOOTING_WHILE_BACK_INTAKING:
            // case PREPARING_SHOOT_WHILE_BACK_INTAKING:
            // case FEEDING_WHILE_BACK_INTAKING:
            // case PREPARING_FEED_WHILE_BACK_INTAKING:
                // superstructure.SetWantedSuperState(WantedSuperState.BACK_INTAKE);
                // break;
// 
            // case SHOOTING_WHILE_DOUBLE_INTAKING:
            // case PREPARING_SHOOT_WHILE_DOUBLE_INTAKING:
            // case FEEDING_WHILE_DOUBLE_INTAKING:
            // case PREPARING_FEED_WHILE_DOUBLE_INTAKING:
                // superstructure.SetWantedSuperState(WantedSuperState.DOUBLE_INTAKE);
                // break;
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
