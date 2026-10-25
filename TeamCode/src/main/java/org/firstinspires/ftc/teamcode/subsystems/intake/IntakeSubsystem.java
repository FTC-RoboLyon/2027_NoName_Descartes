package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO intakeIO;

    DoubleSupplier powerSupplier;

    private double motorPower = 0.0;
    public enum WantedState
    {
        STAND_BY,
        INTAKE
    }

    public enum SystemState
    {
        IDLE,
        INTAKING
    }

    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState =  SystemState.IDLE;
    private ControlMode controlMode = ControlMode.DISABLED;

    public IntakeSubsystem(HardwareMap hmap, boolean isFrontIntake)
    {
        intakeIO = new IntakeIO(hmap, isFrontIntake);
    }

    public void SetControlMode (ControlMode mode)
    {
        this.controlMode = mode;
    }

    public void SetWantedState(WantedState wantedState)
    {
        this.wantedState = wantedState;
    }

    public void ConfigureManualSupplier(DoubleSupplier powerSupplier)
    {
        this.powerSupplier = powerSupplier;
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

            case MANUAL_VOLTAGE:
                if (powerSupplier != null)
                {
                    motorPower = powerSupplier.getAsDouble();
                }
                break;

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
