package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO intakeIO;

    DoubleSupplier powerSupplier;

    private double frontMotorPower = 0.0;
    private double backMotorPower = 0.0;

    public enum WantedState
    {
        STAND_BY,
        FRONT_INTAKE,
        BACK_INTAKE,
        CRAZY_INTAKING
    }

    public enum SystemState
    {
        IDLE,
        FRONT_INTAKING,
        BACK_INTAKING,
        EATING_BATTERY_POWER
    }

    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState =  SystemState.IDLE;
    private ControlMode controlMode = ControlMode.DISABLED;

    public IntakeSubsystem(IntakeIO intakeIO)
    {
        this.intakeIO = intakeIO;
    }

    public void SetControlMode(ControlMode mode)
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

        public SystemState getSystemState()
    {
        return systemState;
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
                frontMotorPower = 0.0;
                backMotorPower = 0.0;
                break;

            case VOLTAGE:
                switch(systemState)
                {
                    case IDLE:
                        frontMotorPower = IntakeConstants.REST;
                        backMotorPower = IntakeConstants.REST;
                        break;

                    case FRONT_INTAKING:
                        frontMotorPower = IntakeConstants.INTAKE_POWER;
                        backMotorPower = IntakeConstants.REST;
                        break;

                    case BACK_INTAKING:
                        frontMotorPower = IntakeConstants.REST;
                        backMotorPower = IntakeConstants.INTAKE_POWER;
                        break;

                    case EATING_BATTERY_POWER:
                        frontMotorPower = IntakeConstants.INTAKE_POWER;
                        backMotorPower = IntakeConstants.INTAKE_POWER;
                        break;

                    default:
                        throw new IllegalStateException("Can't run Intake state machine with an unknown systemState");
                }
                break;

            case MANUAL_VOLTAGE:
                if (powerSupplier != null)
                {
                    frontMotorPower = powerSupplier.getAsDouble();
                    backMotorPower = powerSupplier.getAsDouble();
                }
                break;

            default:
                throw new IllegalStateException("Unsupported Control Mode used for intake");
        }

        intakeIO.SetPower(frontMotorPower, backMotorPower);
    }

    public void RunStateMachine()
    {
        switch(wantedState)
        {
            case STAND_BY:
                systemState = SystemState.IDLE;
                break;

            case FRONT_INTAKE:
                systemState = SystemState.FRONT_INTAKING;
                break;

            case BACK_INTAKE:
                systemState = SystemState.BACK_INTAKING;
                break;

            case CRAZY_INTAKING:
                systemState = SystemState.EATING_BATTERY_POWER;
                break;

            default:
                throw new IllegalStateException("Can't run Intake State machine with an unknown wantedState");
        }

        switch(systemState)
        {
            case IDLE:
                break;

            case FRONT_INTAKING:
            case BACK_INTAKING:
            case EATING_BATTERY_POWER:
                //TODO: stop when 3 ball
                break;

            default:
                throw new IllegalStateException("Can't run Intake State machine with an unknown systemState");
        }
    }
}
