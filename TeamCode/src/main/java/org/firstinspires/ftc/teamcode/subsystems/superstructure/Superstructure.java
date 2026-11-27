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
        FRONT_INTAKE,
        BACK_INTAKE,
        DOUBLE_INTAKE,
        SHOOT,
        FEED_ALLIANCE_ZONE,
        //Prepare states
        PREPARE_SHOOT,
        PREPARE_FEED,
        //Combined states
        SHOOT_FRONT_INTAKING,
        SHOOT_BACK_INTAKING,
        SHOOT_DOUBLE_INTAKING,

        FEED_FRONT_INTAKING,
        FEED_BACK_INTAKING,
        FEED_DOUBLE_INTAKING,

        PREPARE_SHOOT_FRONT_INTAKING,
        PREPARE_SHOOT_BACK_INTAKING,
        PREPARE_SHOOT_DOUBLE_INTAKING,
        //Optional test TODO
        // POLINIZE_FLOWERS,
        // SHOOT_WITH_INTAKE,
        // DEPOLINIZE_FLOWERS
    }
    
    public enum SystemSuperState
    {
        IDLE,
        //Steady States
        FRONT_INTAKING,
        BACK_INTAKING,
        DOUBLE_INTAKING,

        SHOOTING,
        FEEDING,

        SHOOTING_WHILE_FRONT_INTAKING,
        SHOOTING_WHILE_BACK_INTAKING,
        SHOOTING_WHILE_DOUBLE_INTAKING,

        FEEDING_WHILE_FRONT_INTAKING,
        FEEDING_WHILE_BACK_INTAKING,
        FEEDING_WHILE_DOUBLE_INTAKING,
        //Transition States
        PREPARING_SHOOT,
        PREPARING_FEED,

        PREPARING_SHOOT_WHILE_FRONT_INTAKING,
        PREPARING_SHOOT_WHILE_BACK_INTAKING,
        PREPARING_SHOOT_WHILE_DOUBLE_INTAKING,

        PREPARING_FEED_WHILE_FRONT_INTAKING,
        PREPARING_FEED_WHILE_BACK_INTAKING,
        PREPARING_FEED_WHILE_DOUBLE_INTAKING,
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STAND_BY;
    private SystemSuperState systemSuperState = SystemSuperState.IDLE;

    private IntakeSubsystem intake;
    private FeederSubsystem feeder;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;

    public void SetWantedSuperState(WantedSuperState wantedSuperState)
    {
        this.wantedSuperState = wantedSuperState;
        //TODO :Combined States
    }

    public SystemSuperState GetSystemSuperState()
    {
        return systemSuperState;
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
        RunSuperStateMachine();

        // switch(systemSuperState)
        // {
            // case IDLE:
                // intake.SetWantedState(IntakeSubsystem.WantedState.STAND_BY);
                // feeder.SetWantedState(FeederSubsystem.WantedState.STAND_BY);
                // turret.SetWantedState(TurretSubsystem.WantedState.STAND_BY);
                // shooter.SetWantedState(ShooterSubsystem.WantedState.STAND_BY);
                // break;
// 
            // case FRONT_INTAKING:
                // intake.SetWantedState(IntakeSubsystem.WantedState.FRONT_INTAKE);
                // feeder.SetWantedState(FeederSubsystem.WantedState.STAND_BY);
                // turret.SetWantedState(TurretSubsystem.WantedState.STAND_BY);
                // shooter.SetWantedState(ShooterSubsystem.WantedState.STAND_BY);
                // break;
// 
            // case BACK_INTAKING:
                // intake.SetWantedState(IntakeSubsystem.WantedState.BACK_INTAKE);
                // feeder.SetWantedState(FeederSubsystem.WantedState.STAND_BY);
                // turret.SetWantedState(TurretSubsystem.WantedState.STAND_BY);
                // shooter.SetWantedState(ShooterSubsystem.WantedState.STAND_BY);
                // break;
// 
            // case DOUBLE_INTAKING:
                // intake.SetWantedState(IntakeSubsystem.WantedState.DOUBLE_INTAKING);
                // feeder.SetWantedState(FeederSubsystem.WantedState.STAND_BY);
                // turret.SetWantedState(TurretSubsystem.WantedState.STAND_BY);
                // shooter.SetWantedState(ShooterSubsystem.WantedState.STAND_BY);
                // break;
// 
            // case SHOOTING:
                // intake.SetWantedState(IntakeSubsystem.WantedState.STAND_BY);
                // feeder.SetWantedState(FeederSubsystem.WantedState.FEED);
                // turret.SetWantedState(TurretSubsystem.WantedState.TRACK_PUBLIC_CELL);
                // shooter.SetWantedState(ShooterSubsystem.WantedState.SHOOT_PUBLIC_CELL);
                // break;
        // }
    }


    // --- private methods ---

    private void RunSuperStateMachine()
    {
        switch(wantedSuperState)
        {
            case STAND_BY:
                break;

            case FRONT_INTAKE:
                systemSuperState = SystemSuperState.FRONT_INTAKING;
                break;

            case BACK_INTAKE:
                systemSuperState = SystemSuperState.BACK_INTAKING;
                break;

            case DOUBLE_INTAKE:
                systemSuperState = SystemSuperState.DOUBLE_INTAKING;
                break;

            case SHOOT:
                if(systemSuperState != SystemSuperState.SHOOTING)
                {
                    systemSuperState = SystemSuperState.PREPARING_SHOOT;
                }
                break;

            case FEED_ALLIANCE_ZONE:
                if(systemSuperState != SystemSuperState.FEEDING)
                {
                    systemSuperState = SystemSuperState.PREPARING_FEED;
                }
                break;

            case PREPARE_SHOOT:
                systemSuperState = SystemSuperState.PREPARING_SHOOT;
                break;

            case PREPARE_FEED:    
                systemSuperState = SystemSuperState.PREPARING_FEED;
                break;

            case SHOOT_FRONT_INTAKING:
                if(systemSuperState != SystemSuperState.SHOOTING_WHILE_FRONT_INTAKING)
                {
                    systemSuperState = SystemSuperState.PREPARING_SHOOT_WHILE_FRONT_INTAKING;
                }
                break;

            case SHOOT_BACK_INTAKING:
                if(systemSuperState != SystemSuperState.SHOOTING_WHILE_BACK_INTAKING)
                {
                    systemSuperState = SystemSuperState.PREPARING_SHOOT_WHILE_BACK_INTAKING;
                }
                break;

            case SHOOT_DOUBLE_INTAKING:
                if(systemSuperState != SystemSuperState.SHOOTING_WHILE_DOUBLE_INTAKING)
                {
                    systemSuperState = SystemSuperState.PREPARING_SHOOT_WHILE_DOUBLE_INTAKING;
                }
                break;

            case FEED_FRONT_INTAKING:
                if(systemSuperState != SystemSuperState.FEEDING_WHILE_FRONT_INTAKING)
                {
                    systemSuperState = SystemSuperState.PREPARING_FEED_WHILE_FRONT_INTAKING;
                }
                break;

            case FEED_BACK_INTAKING:
                if(systemSuperState != SystemSuperState.FEEDING_WHILE_BACK_INTAKING)
                {
                    systemSuperState = SystemSuperState.PREPARING_FEED_WHILE_BACK_INTAKING;
                }
                break;

            case FEED_DOUBLE_INTAKING:
                if(systemSuperState != SystemSuperState.FEEDING_WHILE_DOUBLE_INTAKING)
                {
                    systemSuperState = SystemSuperState.PREPARING_FEED_WHILE_DOUBLE_INTAKING;
                }
                break;

            case PREPARE_SHOOT_FRONT_INTAKING:
                systemSuperState = SystemSuperState.PREPARING_SHOOT_WHILE_FRONT_INTAKING;
                break;

            case PREPARE_SHOOT_BACK_INTAKING:
                systemSuperState = SystemSuperState.PREPARING_SHOOT_WHILE_BACK_INTAKING;
                break;

            case PREPARE_SHOOT_DOUBLE_INTAKING:
                systemSuperState = SystemSuperState.PREPARING_SHOOT_WHILE_DOUBLE_INTAKING;
                break;

            default:
                throw new IllegalStateException("The robot can't be sth he don't know (problem in Superstructure)");
        }

        switch(systemSuperState)
        {
            case IDLE:
            case FRONT_INTAKING:
            case BACK_INTAKING:
            case DOUBLE_INTAKING:
            case SHOOTING:
            case FEEDING:
            case SHOOTING_WHILE_FRONT_INTAKING:
            case SHOOTING_WHILE_BACK_INTAKING:
            case SHOOTING_WHILE_DOUBLE_INTAKING:
            case FEEDING_WHILE_FRONT_INTAKING:
            case FEEDING_WHILE_BACK_INTAKING:
            case FEEDING_WHILE_DOUBLE_INTAKING:
                break;

            case PREPARING_SHOOT:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedSuperState == WantedSuperState.SHOOT)
                    {
                        systemSuperState = SystemSuperState.SHOOTING;
                    }
                break;

            case PREPARING_FEED: 
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedSuperState == WantedSuperState.FEED_ALLIANCE_ZONE)
                    {
                        systemSuperState = SystemSuperState.FEEDING;
                    }
                break;

            case PREPARING_SHOOT_WHILE_FRONT_INTAKING:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedSuperState == WantedSuperState.SHOOT_FRONT_INTAKING)
                    {
                        systemSuperState = SystemSuperState.SHOOTING_WHILE_FRONT_INTAKING;
                    }
                break;

            case PREPARING_SHOOT_WHILE_BACK_INTAKING:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedSuperState == WantedSuperState.SHOOT_BACK_INTAKING)
                    {
                        systemSuperState = SystemSuperState.SHOOTING_WHILE_BACK_INTAKING;
                    }
                break;

            case PREPARING_SHOOT_WHILE_DOUBLE_INTAKING:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedSuperState == WantedSuperState.SHOOT_DOUBLE_INTAKING)
                    {
                        systemSuperState = SystemSuperState.SHOOTING_WHILE_DOUBLE_INTAKING;
                    }
                break;

            case PREPARING_FEED_WHILE_FRONT_INTAKING:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedSuperState == WantedSuperState.FEED_FRONT_INTAKING)
                    {
                        systemSuperState = SystemSuperState.FEEDING_WHILE_FRONT_INTAKING;
                    }
                break;

            case PREPARING_FEED_WHILE_BACK_INTAKING:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedSuperState == WantedSuperState.FEED_BACK_INTAKING)
                    {
                        systemSuperState = SystemSuperState.FEEDING_WHILE_BACK_INTAKING;
                    }
                break;

            case PREPARING_FEED_WHILE_DOUBLE_INTAKING:
                if(shooter.getSystemState() == ShooterSubsystem.SystemState.READY_TO_SHOOT_OPPOSITE_CELL //TODO : correct when better state machine
                    && turret.getSystemState() == TurretSubsystem.SystemState.ALIGNED_WITH_OPPOSITE_CELL
                    && wantedSuperState == WantedSuperState.FEED_DOUBLE_INTAKING)
                    {
                        systemSuperState = SystemSuperState.FEEDING_WHILE_DOUBLE_INTAKING;
                    }
                break;

            default:
                throw new IllegalStateException("Superstructure doesn't understand its own state");
        }
    }
}
