package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO intakeIO;

    private double motorPower = 0.0;
    private enum WantedState
    {
        STAND_BY,
        INTAKE
    }

    private enum SystemState
    {
        IDLE,
        INTAKING
    }

    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState =  SystemState.IDLE;
    private ControlMode controlMode = ControlMode.DISABLED;

    public IntakeSubsystem(HardwareMap hmap)
    {
        intakeIO = new IntakeIO(hmap);
    }

    @Override
    public void periodic()
    {
        if (controlMode.allowsStateMachine())
        {
            RunStateMachine();
        }

        switch(controlMode)
        {
            case DISABLED:
                motorPower = 0.0;
                break;

            case VOLTAGE:
                switch(systemState)
                {
                    case IDLE:
                        motorPower = intakeConstants.REST;
                        break;

                    case INTAKING:
                        motorPower = intakeConstants.INTAKE_POWER;
                        break;

                    default:
                        //TODO
                        break;
                }

            default:
                //TODO
                break;
        }

        intakeIO.SetPower(motorPower);
    }

    public void RunStateMachine()
    {
        switch(wantedState)
        {
            case STAND_BY:
                systemState = SystemState.IDLE;
                break;

            case INTAKE:
                systemState = SystemState.INTAKING;
                break;

            default:
                //TODO
                break;
        }

        switch(systemState)
        {
            case IDLE:
                break;

            case INTAKING:
                //TODO: stop when 3 ball
                break;

            default:
                //TODO : warn
                break;
        }
    }
}
