package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.superstructure.Superstructure;

public class SetWantedSuperStateCmd extends CommandBase{

    private Superstructure superstructure;
    private Superstructure.WantedSuperState wantedSuperState;
    private Superstructure.WantedSuperState adaptedWantedSuperState;

    public SetWantedSuperStateCmd(Superstructure superstructure, Superstructure.WantedSuperState wantedSuperState)
    {
        this.superstructure = superstructure;
        this.wantedSuperState = wantedSuperState;
    }

    @Override
    public void initialize()
    {
        switch(wantedSuperState)
        {
            case STAND_BY:
                break;

            case FRONT_INTAKE:
                
        }
    }

    @Override 
    public boolean isFinished()
    {
        return true;
    }
}
