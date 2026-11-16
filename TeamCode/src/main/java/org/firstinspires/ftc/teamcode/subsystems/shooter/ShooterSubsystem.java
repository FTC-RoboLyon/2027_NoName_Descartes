package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.teamcode.subsystems.shooterParametersCalculator.ShooterParametersCalculator;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {

    private HoodIO hoodIO;
    private FlywheelIO flywheelIO;
    private ShooterParametersCalculator shooterParametersCalculator;

    private PidRBL flywheelPID = new PidRBL(FlywheelConstants.FLYWHEEL_MOTOR_KP, FlywheelConstants.FLYWHEEL_MOTOR_KI, FlywheelConstants.FLYWHEEL_MOTOR_KD);
    private FeedForwardModel flywheelFeedForward = new FeedForwardModel(FlywheelConstants.FLYWHEEL_MOTOR_KS, FlywheelConstants.FLYWHEEL_MOTOR_KV, 0.0);

    private PidRBL hoodPID = new PidRBL(HoodConstants.HOOD_KP, HoodConstants.HOOD_KI, HoodConstants.HOOD_KD);

    private BooleanSupplier shootNearTrigger, shootFacedWithCellTrigger, shootFromFlowerTrigger = ()-> false;
    private DoubleSupplier flywheelManualVoltage = ()-> 0.0;

    private double flywheelTargetSpeed = 0.0;
    private double hoodTargetAngle = 0.0;
    private boolean isAtTargetSpeed = false;
    private boolean isAtTargetAngle = false;

    public enum WantedState
    {
        STAND_BY,
        STOP,

        SHOOT_PUBLIC_CELL,
        SHOOT_OPPOSITE_CELL,
        SHOOT_PUBLIC_SIDE,
        SHOOT_OPPOSITE_SIDE
    }

    public enum SystemState
    {
        IDLE,
        BRAKING_DOWN,
        STOPPED,

        PREPARING_SHOOT_PUBLIC_CELL,
        PREPARING_SHOOT_OPPOSITE_CELL,

        PREPARING_SHOOT_PUBLIC_SIDE,
        PREPARING_SHOOT_OPPOSITE_SIDE,


        READY_TO_SHOOT_PUBLIC_CELL,
        READY_TO_SHOOT_OPPOSITE_CELL,

        READY_TO_SHOOT_PUBLIC_SIDE,
        READY_TO_SHOOT_OPPOSITE_SIDE
    }

    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState = ShooterSubsystem.SystemState.IDLE;

    public SystemState getSystemState() {return systemState;}
    public void setWantedState(WantedState wantedState){this.wantedState = wantedState;}


    private ControlMode flywheelControlMode = ControlMode.MANUAL_VELOCITY;
    private ControlMode hoodControlMode = ControlMode.MANUAL_POSITION;

    public ControlMode getFlywheelControlMode() {return flywheelControlMode;}
    public void setFlywheelControlMode(ControlMode flywheelControlMode) {this.flywheelControlMode = flywheelControlMode;}

    public ControlMode getHoodControlMode() {return hoodControlMode;}
    public void setHoodControlMode(ControlMode hoodControlMode) {this.hoodControlMode = hoodControlMode;}


    public ShooterSubsystem(HoodIO hoodIO, FlywheelIO flywheelIO, ShooterParametersCalculator shooterParametersCalculator)
    {
        this.hoodIO = hoodIO;
        this.flywheelIO = flywheelIO;
        this.shooterParametersCalculator = shooterParametersCalculator;

        flywheelPID.setOutputLimits(-1.0, 1.0);
        flywheelFeedForward.setOutputLimits(-1.0, 1.0);

        hoodPID.setOutputLimits(-1.0, 1.0);

    }

    public void setSuppliersManualSpeedSuppliers(BooleanSupplier shootNearTrigger, BooleanSupplier shootFacedWithCellTrigger, BooleanSupplier shootFromFlowerTrigger)
    {
        this.shootNearTrigger = shootNearTrigger;
        this.shootFacedWithCellTrigger = shootFacedWithCellTrigger;
        this.shootFromFlowerTrigger = shootFromFlowerTrigger;
    }

    public void setFlywheelManualVoltageSupplier(DoubleSupplier flywheelManualVoltage)
    {
        this.flywheelManualVoltage = flywheelManualVoltage;
    }


    @Override
    public void periodic()
    {
        flywheelIO.update();
        hoodIO.update();

        if (hoodControlMode.allowsStateMachine() || flywheelControlMode.allowsStateMachine())
            RunStateMachine();

        switch (hoodControlMode)
        {
            case MANUAL_POSITION:

                if (shootNearTrigger.getAsBoolean())
                    hoodTargetAngle = HoodConstants.HOOD_ANGLE_NEAR;
                else if (shootFacedWithCellTrigger.getAsBoolean())
                    hoodTargetAngle = HoodConstants.HOOD_ANGLE_FACED_WITH_CELL;
                else if (shootFromFlowerTrigger.getAsBoolean())
                    hoodTargetAngle = HoodConstants.HOOD_ANGLE_FROM_FLOWER;

                hoodIO.setVoltage(
                        hoodPID.calculateWithRealTime(hoodTargetAngle, hoodIO.getHoodAngle(), System.nanoTime())
                );
                break;

            case MANUAL_DUTY_CYCLE:
                break;
            case POSITION_DUTYCYCLE_PID:
                switch (systemState)
                {
                    case IDLE:
                        break;

                    case BRAKING_DOWN:
                    case STOPPED:
                    case PREPARING_SHOOT_PUBLIC_CELL:
                    case READY_TO_SHOOT_PUBLIC_CELL:
                    case PREPARING_SHOOT_OPPOSITE_CELL:
                    case READY_TO_SHOOT_OPPOSITE_CELL:
                    case PREPARING_SHOOT_PUBLIC_SIDE:
                    case READY_TO_SHOOT_PUBLIC_SIDE:
                    case PREPARING_SHOOT_OPPOSITE_SIDE:
                    case READY_TO_SHOOT_OPPOSITE_SIDE:
                        hoodIO.setVoltage(
                                hoodPID.calculate(hoodTargetAngle, hoodIO.getHoodAngle())
                        );
                        break;

                    default:
                        throw new IllegalStateException("System state isn(t valid");
                }
                break;
            case DISABLED:
                break;
            default:
                throw new IllegalStateException("Unsupported Control Mode used for hood");
        }

        switch (flywheelControlMode)
        {
            case MANUAL_VELOCITY:
                if (shootNearTrigger.getAsBoolean())
                    flywheelTargetSpeed = FlywheelConstants.FLYWHEEL_SPEED_NEAR;
                else if (shootFacedWithCellTrigger.getAsBoolean())
                    flywheelTargetSpeed = FlywheelConstants.FLYWHEEL_SPEED_FACED_WITH_CELL;
                else if (shootFromFlowerTrigger.getAsBoolean())
                    flywheelTargetSpeed = FlywheelConstants.FLYWHEEL_SPEED_FROM_FLOWER;

                flywheelIO.applyPower(
                        flywheelFeedForward.calculate(flywheelTargetSpeed)
                                + flywheelPID.calculate(flywheelTargetSpeed, flywheelIO.getFlywheelVelocity())
                );
                break;
            case MANUAL_DUTY_CYCLE:
                flywheelIO.applyPower(flywheelManualVoltage.getAsDouble());
                break;
            case VELOCITY_DUTYCYCLE_PID_FF:
                switch (systemState)
                {
                    case IDLE:
                        break;

                    case BRAKING_DOWN:
                    case STOPPED:
                        flywheelIO.applyPower(0.0);
                        break;

                    case PREPARING_SHOOT_PUBLIC_CELL:
                    case READY_TO_SHOOT_PUBLIC_CELL:
                    case PREPARING_SHOOT_OPPOSITE_CELL:
                    case READY_TO_SHOOT_OPPOSITE_CELL:
                    case PREPARING_SHOOT_PUBLIC_SIDE:
                    case READY_TO_SHOOT_PUBLIC_SIDE:
                    case PREPARING_SHOOT_OPPOSITE_SIDE:
                    case READY_TO_SHOOT_OPPOSITE_SIDE:
                        flywheelIO.applyPower(
                                flywheelFeedForward.calculate(flywheelTargetSpeed)
                                        + flywheelPID.calculate(flywheelTargetSpeed, flywheelIO.getFlywheelVelocity())
                        );
                        break;

                    default:
                        throw new IllegalStateException("System state isn(t valid");
                }
                break;
            case DISABLED:
                break;
            default:
                throw new IllegalStateException("Unsupported Control Mode used for hood");
        }
    }

    private void RunStateMachine()
    {
        switch (wantedState)
        {
            case STAND_BY:

                if (systemState != SystemState.IDLE)
                    systemState = SystemState.IDLE;
                break;

            case STOP:

                if (systemState != SystemState.STOPPED)
                    systemState = SystemState.BRAKING_DOWN;

                flywheelTargetSpeed = 0.0;
                hoodTargetAngle = HoodConstants.HOOD_DEFAULT_POS;
                break;

            case SHOOT_PUBLIC_CELL:

                if (systemState != SystemState.READY_TO_SHOOT_PUBLIC_CELL)
                    systemState = SystemState.PREPARING_SHOOT_PUBLIC_CELL;

                flywheelTargetSpeed = shooterParametersCalculator.getShootParam().flywheelSpeed;
                hoodTargetAngle = shooterParametersCalculator.getShootParam().hoodAngle;
                break;

            case SHOOT_OPPOSITE_CELL:

                if (systemState != SystemState.READY_TO_SHOOT_OPPOSITE_CELL)
                    systemState = SystemState.PREPARING_SHOOT_OPPOSITE_CELL;

                flywheelTargetSpeed = shooterParametersCalculator.getShootParam().flywheelSpeed;
                hoodTargetAngle = shooterParametersCalculator.getShootParam().hoodAngle;
                break;

            case SHOOT_PUBLIC_SIDE:

                if (systemState != SystemState.READY_TO_SHOOT_PUBLIC_SIDE)
                    systemState = SystemState.PREPARING_SHOOT_PUBLIC_SIDE;

                flywheelTargetSpeed = shooterParametersCalculator.getShootParam().flywheelSpeed;
                hoodTargetAngle = shooterParametersCalculator.getShootParam().hoodAngle;
                break;

            case SHOOT_OPPOSITE_SIDE:

                if (systemState != SystemState.READY_TO_SHOOT_OPPOSITE_SIDE)
                    systemState = SystemState.PREPARING_SHOOT_OPPOSITE_SIDE;

                flywheelTargetSpeed = shooterParametersCalculator.getShootParam().flywheelSpeed;
                hoodTargetAngle = shooterParametersCalculator.getShootParam().hoodAngle;
                break;

            default:
                throw new IllegalStateException("Please enter a valid shooter wanted state");
        }

        isAtTargetSpeed = UtilsRBL.isInRange(flywheelIO.getFlywheelVelocity(), flywheelTargetSpeed, FlywheelConstants.FLYWHEEL_TOLERANCE);
        isAtTargetAngle = UtilsRBL.isInRange(hoodIO.getHoodAngle(), hoodTargetAngle, HoodConstants.HOOD_TOLERANCE);

        switch (systemState)
        {
            case IDLE:
            case STOPPED:
                break;

            case BRAKING_DOWN:
                if (UtilsRBL.isInRange(flywheelIO.getFlywheelVelocity(), 0.0, FlywheelConstants.FLYWHEEL_TOLERANCE))
                    systemState = SystemState.STOPPED;
                break;

            case PREPARING_SHOOT_PUBLIC_CELL:
                if (isAtTargetSpeed && isAtTargetAngle)
                    systemState = SystemState.READY_TO_SHOOT_PUBLIC_CELL;
                break;

            case PREPARING_SHOOT_OPPOSITE_CELL:
                if (isAtTargetSpeed && isAtTargetAngle)
                    systemState = SystemState.READY_TO_SHOOT_OPPOSITE_CELL;
                break;

            case PREPARING_SHOOT_PUBLIC_SIDE:
                if (isAtTargetSpeed && isAtTargetAngle)
                    systemState = SystemState.READY_TO_SHOOT_PUBLIC_SIDE;
                break;

            case PREPARING_SHOOT_OPPOSITE_SIDE:
                if (isAtTargetSpeed && isAtTargetAngle)
                    systemState = SystemState.READY_TO_SHOOT_OPPOSITE_SIDE;
                break;

            case READY_TO_SHOOT_PUBLIC_CELL:
            case READY_TO_SHOOT_OPPOSITE_CELL:
            case READY_TO_SHOOT_PUBLIC_SIDE:
            case READY_TO_SHOOT_OPPOSITE_SIDE:
            default:
                throw new IllegalStateException("System state isn(t valid");
        }
    }

}
