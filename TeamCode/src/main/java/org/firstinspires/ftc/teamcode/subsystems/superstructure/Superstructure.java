package org.firstinspires.ftc.teamcode.subsystems.superstructure;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;

import org.firstinspires.ftc.teamcode.subsystems.feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.feeder.FeederConstants;

import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelConstants;


public class Superstructure extends SubsystemBase{
    public enum WantedSuperState
    {
        STAND_BY,
        //Basic states
        INTAKE,
        SHOOT,
        FEED_ALLIANCE_ZONE,
        //Prepare states
        PREPARE_SHOOT,
        PREPARE_FEED,
        //Combined states
        SHOOT_INTAKING,
        FEED_INTAKING,
        PREPARE_SHOOT_INTAKING,
        //Optional test TODO
        // POLINIZE_FLOWERS,
        // SHOOT_WITH_INTAKE,
        // DEPOLINIZE_FLOWERS
    }
    
    public enum SystemSuperState
    {
        IDLE,
        //Steady States
        INTAKING,
        SHOOTING,
        FEEDING,
        SHOOTING_WHILE_INTAKING,
        FEEDING_WHILE_INTAKING,
        //Transition States
        PREPARING_SHOOT,
        PREPARING_FEED,
        PREPARING_SHOOT_WHILE_INTAKING,
        PREPARING_FEED_WHILE_INTAKING
    }

    private WantedSuperState wantedState = WantedSuperState.STAND_BY;
    private SystemSuperState systemState = SystemSuperState.IDLE;

    private IntakeSubsystem intake;
    private FeederSubsystem feeder;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;

    public void SetWantedSuperState(WantedSuperState wantedState)
    {
        this.wantedState = wantedState;
        //TODO :Combined States
    }

    public SystemSuperState GetSystemSuperState()
    {
        return systemState;
    }

    // --- Subsystems Control Mode ---

    public void EnableSubsystems()
    {
        intake.SetControlMode(IntakeConstants.MainControlMode);
        feeder.SetControlMode(FeederConstants.MainControlMode);
        //turret.SetControlMode(TurretConstants.MainControlMode);
        shooter.setFlywheelControlMode(FlywheelConstants.MainConntrolMode);
        shooter.setHoodControlMode(HoodConstants.MainConntrolMode);
    }

    public void DisableSubsystems()
    {
        intake.SetControlMode(ControlMode.DISABLED);
        feeder.SetControlMode(ControlMode.DISABLED);
        //turret.SetControlMode(ControlMode.DISABLED);
        shooter.setFlywheelControlMode(ControlMode.DISABLED);
        shooter.setHoodControlMode(ControlMode.DISABLED);
    }

    //TODO : Toggle Control Mode fucntions


    @Override 
    public void periodic()
    {

    }


    // --- private methods ---

    private void RunSuperStateMachine()
    {
        switch(wantedState)
        {
            case STAND_BY:
                break;

            case INTAKE:
                systemState = SystemSuperState.INTAKING;
                break;

            case SHOOT:
                if(systemState != SystemSuperState.SHOOTING)
                {
                    systemState = SystemSuperState.PREPARING_SHOOT;
                }
                break;

            case FEED_ALLIANCE_ZONE:
                if(systemState != SystemSuperState.FEEDING)
                {
                    systemState = SystemSuperState.PREPARING_FEED;
                }
                break;

            case PREPARE_SHOOT:
                systemState = SystemSuperState.PREPARING_SHOOT;
                break;

            case PREPARE_FEED:    
                systemState = SystemSuperState.PREPARING_FEED;
                break;

            case SHOOT_INTAKING:
                if(systemState != SystemSuperState.SHOOTING_WHILE_INTAKING)
                {
                    systemState = SystemSuperState.PREPARING_SHOOT_WHILE_INTAKING;
                }
                break;

            case FEED_INTAKING:
                if(systemState != SystemSuperState.FEEDING_WHILE_INTAKING)
                {
                    systemState = SystemSuperState.PREPARING_FEED_WHILE_INTAKING;
                }
                break;

            case PREPARE_SHOOT_INTAKING:
                systemState = SystemSuperState.PREPARING_SHOOT_WHILE_INTAKING;
                break;

            default:
                throw new IllegalStateException("The robot can't be sth he don't know (problem in Superstructure)");
        }

        switch(systemState)
        {
            case IDLE:
            case INTAKING:
            case SHOOTING:
            case FEEDING:
            case SHOOTING_WHILE_INTAKING:
            case FEEDING_WHILE_INTAKING:
                break;

            case PREPARING_SHOOT:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedState == WantedSuperState.SHOOT)
                    {
                        systemState = SystemSuperState.SHOOTING;
                    }
                break;

            case PREPARING_FEED: 
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedState == WantedSuperState.FEED_ALLIANCE_ZONE)
                    {
                        systemState = SystemSuperState.FEEDING;
                    }
                break;

            case PREPARING_SHOOT_WHILE_INTAKING:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedState == WantedSuperState.SHOOT_INTAKING)
                    {
                        systemState = SystemSuperState.SHOOTING_WHILE_INTAKING;
                    }
                break;

            case PREPARING_FEED_WHILE_INTAKING:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedState == WantedSuperState.FEED_INTAKING)
                    {
                        systemState = SystemSuperState.FEEDING_WHILE_INTAKING;
                    }
                break;

            default:
                throw new IllegalStateException("Superstructure doesn't understand its own state");
        }
    }
}
