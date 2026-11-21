package org.firstinspires.ftc.teamcode.subsystems.feeder;

import org.firstinspires.ftc.teamcode.subsystems.feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.feeder.FeederIO;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

import com.seattlesolvers.solverslib.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase{
    public enum WantedState
    {
        STAND_BY,
        FEED
    }

    public enum SystemState
    {
        IDLE,
        FEEDING,
        SLEEPING
    }

    private FeederIO feederIO;

    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState = SystemState.IDLE;
    private ControlMode controlMode = ControlMode.DISABLED;

    private double motorPower = 0.0;

    public FeederSubsystem(FeederIO io)
    {
        this.feederIO = io;
    }

    public void SetControlMode(ControlMode controlMode)
    {
        this.controlMode = controlMode;
    }

    public void SetWantedState(WantedState wantedState)
    {
        this.wantedState = wantedState;
    }

    public SystemState getSystemState()
    {
        return systemState;
    }

    @Override 
    public void periodic()
    {
        if(controlMode.allowsStateMachine())
        {
            RunStateMachine();
        }

        switch(controlMode)
        {
            case DISABLED:
                motorPower = 0.0;
                break;

            case DUTY_CYCLE:
                switch(systemState)
                {
                    case IDLE:
                    case SLEEPING:
                        motorPower = 0.0;
                        break;

                    case FEEDING:
                        motorPower = FeederConstants.FEEDING_POWER;
                        break;

                    default:
                        throw new IllegalArgumentException("Unknown Feeder Wanted State used");  
                }
                break;

            case MANUAL_DUTY_CYCLE:
                //TODO
                break;

            default:
                throw new IllegalArgumentException("Unknown Feeder ControlMode used"); 
        }

        switch(controlMode)
        {
            case DISABLED:
            case DUTY_CYCLE:
            case MANUAL_DUTY_CYCLE:
                feederIO.setMotorPower(motorPower);
                break;

            default:
                throw new IllegalArgumentException("Unknown Feeder ControlMode used"); 
        }
    }

    private void RunStateMachine()
    {
        switch(wantedState)
        {
            case STAND_BY :
                systemState = SystemState.SLEEPING;
                break;

            case FEED:
                if(systemState != SystemState.FEEDING)
                {
                    systemState = SystemState.FEEDING;
                }
                break;

            default:
                throw new IllegalArgumentException("Unknown Feeder Wanted State used");
        }

        switch(systemState)
        {
            case IDLE:
                systemState = SystemState.SLEEPING;
                break;

            case SLEEPING:
            case FEEDING:
                break;

            default:
                throw new IllegalArgumentException("Unknown Feeder Wanted State used");
        }
    }
}
