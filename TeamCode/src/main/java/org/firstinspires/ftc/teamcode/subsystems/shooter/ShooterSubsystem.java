package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterParametersCalculator.ShooterParametersCalculator;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodIO;

import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {

    private FlywheelIO flywheelIO;
    private HoodIO hoodIO;

    private ShooterParametersCalculator parametersCalculator;

    private Telemetry telemetry;

    private DoubleSupplier leftTrigger;
    private DoubleSupplier rightTrigger;

    private PidRBL flywheelPid = new PidRBL();
    private FeedForwardModel flywheelFF = new FeedForwardModel();
    private ElapsedTime pidTimer;

    private double targetFlywheelSpeed = 0.0;
    private double targetHoodAngle = HoodConstants.HOOD_DEFAULT_POS;
    public enum WantedState
    {
        STAND_BY,
        STOP,

        SHOOT_POLLEN_NEAR,
        SHOOT_POLLEN_MID,
        SHOOT_POLLEN_FAR,
        SHOOT_POLLEN_AUTO,

        SHOOT_NECTAR_NEAR,
        SHOOT_NECTAR_MID,
        SHOOT_NECTAR_FAR,
        SHOOT_NECTAR_AUTO,
    }
    public enum SystemState
    {
        IDLE,
        STOP,

        PREPARING_POLLEN_SHOOT_NEAR,
        PREPARING_POLLEN_SHOOT_MID,
        PREPARING_POLLEN_SHOOT_FAR,
        PREPARING_POLLEN_SHOOT_AUTO,

        PREPARING_NECTAR_SHOOT_NEAR,
        PREPARING_NECTAR_SHOOT_MID,
        PREPARING_NECTAR_SHOOT_FAR,
        PREPARING_NECTAR_SHOOT_AUTO,


        READY_TO_SHOOT_POLLEN_NEAR,
        READY_TO_SHOOT_POLLEN_MID,
        READY_TO_SHOOT_POLLEN_FAR,
        READY_TO_SHOOT_POLLEN_AUTO,

        READY_TO_SHOOT_NECTAR_NEAR,
        READY_TO_SHOOT_NECTAR_MID,
        READY_TO_SHOOT_NECTAR_FAR,
        READY_TO_SHOOT_NECTAR_AUTO,
    }

    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState = SystemState.IDLE;
    private ControlMode controlMode = ControlMode.VELOCITY_DUTYCYCLE_PID_FF;

    public void setWantedState(WantedState state){wantedState = state;}
    public void setShooterControlMode(ControlMode mode){controlMode = mode;}
    public SystemState getSystemState(){return systemState;}

    public ShooterSubsystem(HardwareMap hmap, Telemetry telemetry, ShooterParametersCalculator parametersCalculator)
    {
        flywheelIO = new FlywheelIO(hmap);
        hoodIO = new HoodIO(hmap);

        this.parametersCalculator = parametersCalculator;
        this.telemetry = telemetry;

        flywheelPid.setGains(FlywheelConstants.FLYWHEEL_KP, FlywheelConstants.FLYWHEEL_KI, FlywheelConstants.FLYWHEEL_KD);
        flywheelFF.setGains(FlywheelConstants.FLYWHEEL_KP, FlywheelConstants.FLYWHEEL_KI, FlywheelConstants.FLYWHEEL_KD);

        flywheelPid.setOutputLimits(-1.0, 1.0);
        flywheelFF.setOutputLimits(-1.0, 1.0);
        pidTimer = new ElapsedTime();

        leftTrigger = ()->0.0;
        rightTrigger = ()->0.0;
    }
    public void setSuppliers(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger)
    {
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
    }

    public void actualisePID() // for tuning
    {
        flywheelPid.setGains(FlywheelConstants.FLYWHEEL_KP, FlywheelConstants.FLYWHEEL_KI, FlywheelConstants.FLYWHEEL_KD);
        flywheelFF.setGains(FlywheelConstants.FLYWHEEL_KV, FlywheelConstants.FLYWHEEL_KA, FlywheelConstants.FLYWHEEL_KS);
    }

    public void start()
    {
        pidTimer.reset();
        pidTimer.startTime();

        flywheelIO.reset();

        flywheelPid.reset(flywheelIO.getFlywheelVelocity());
    }

    public void resetPID()
    {
        pidTimer.reset();
        flywheelPid.reset(flywheelIO.getFlywheelVelocity());
    }

    @Override
    public void periodic()
    {
        switch (controlMode)
        {
            case DISABLED:

                flywheelIO.applyPower(0.0);
                hoodIO.setPollenisatorPosition(HoodConstants.POLLENISATOR_RESTING_POS);
                hoodIO.setNectarisatorPosition(HoodConstants.NECTARISATOR_RESTING_POS);
                hoodIO.setHoodPosition(HoodConstants.HOOD_DEFAULT_POS);

                break;
            case MANUAL_DUTY_CYCLE:
                flywheelIO.applyPower(rightTrigger.getAsDouble() - leftTrigger.getAsDouble());
                break;
            case VELOCITY_DUTYCYCLE_PID_FF:
                RunStateMachine();

                flywheelPid.setSetpoint(targetFlywheelSpeed);
                hoodIO.setHoodPosition(targetHoodAngle);

                switch (systemState)
                {
                    case IDLE:
                        break;
                    case STOP:

                        flywheelIO.applyPower(0.0);
                        hoodIO.setPollenisatorPosition(HoodConstants.POLLENISATOR_RESTING_POS);
                        hoodIO.setNectarisatorPosition(HoodConstants.NECTARISATOR_RESTING_POS);

                        break;

                    case PREPARING_POLLEN_SHOOT_NEAR:
                    case READY_TO_SHOOT_POLLEN_NEAR:
                    case PREPARING_POLLEN_SHOOT_MID:
                    case READY_TO_SHOOT_POLLEN_MID:
                    case PREPARING_POLLEN_SHOOT_FAR:
                    case READY_TO_SHOOT_POLLEN_FAR:
                    case PREPARING_POLLEN_SHOOT_AUTO:
                    case READY_TO_SHOOT_POLLEN_AUTO:

                        flywheelIO.applyPower(flywheelPid.calculateWithRealTime(flywheelIO.getFlywheelVelocity(), pidTimer.milliseconds()) +
                                flywheelFF.calculate(targetFlywheelSpeed));

                        hoodIO.setNectarisatorPosition(HoodConstants.NECTARISATOR_POLLEN_POS);
                        hoodIO.setPollenisatorPosition(HoodConstants.POLLENISATOR_POLLEN_POS);

                        break;

                    case PREPARING_NECTAR_SHOOT_NEAR:
                    case READY_TO_SHOOT_NECTAR_NEAR:
                    case PREPARING_NECTAR_SHOOT_MID:
                    case READY_TO_SHOOT_NECTAR_MID:
                    case PREPARING_NECTAR_SHOOT_FAR:
                    case READY_TO_SHOOT_NECTAR_FAR:
                    case PREPARING_NECTAR_SHOOT_AUTO:
                    case READY_TO_SHOOT_NECTAR_AUTO:

                        flywheelIO.applyPower(flywheelPid.calculateWithRealTime(flywheelIO.getFlywheelVelocity(), pidTimer.milliseconds()) +
                                flywheelFF.calculate(targetFlywheelSpeed));

                        hoodIO.setNectarisatorPosition(HoodConstants.NECTARISATOR_NECTAR_POS);
                        hoodIO.setPollenisatorPosition(HoodConstants.POLLENISATOR_NECTAR_POS);

                        break;

                    default:
                        telemetry.addLine("System state isn't valid ");
                }
                break;
        }
    }
    private void RunStateMachine()
    {
        switch (wantedState)
        {
            case STAND_BY:

                systemState = SystemState.IDLE;

                break;
            case STOP:

                if (systemState != SystemState.STOP)
                    flywheelPid.reset(flywheelIO.getFlywheelVelocity());

                targetHoodAngle = HoodConstants.HOOD_DEFAULT_POS;

                systemState = SystemState.STOP;
                break;

            case SHOOT_POLLEN_NEAR:

                targetFlywheelSpeed = FlywheelConstants.FLYWHEEL_TARGET_SPEED_POLLEN_NEAR;
                targetHoodAngle = HoodConstants.HOOD_TARGET_POS_POLLEN_NEAR;

                if (systemState != SystemState.READY_TO_SHOOT_POLLEN_NEAR)
                    systemState = SystemState.PREPARING_POLLEN_SHOOT_NEAR;

                break;

            case SHOOT_POLLEN_MID:

                targetFlywheelSpeed = FlywheelConstants.FLYWHEEL_TARGET_SPEED_POLLEN_MID;
                targetHoodAngle = HoodConstants.HOOD_TARGET_POS_POLLEN_MID;

                if (systemState != SystemState.READY_TO_SHOOT_POLLEN_MID)
                    systemState = SystemState.PREPARING_POLLEN_SHOOT_MID;

                break;

            case SHOOT_POLLEN_FAR:

                targetFlywheelSpeed = FlywheelConstants.FLYWHEEL_TARGET_SPEED_POLLEN_FAR;
                targetHoodAngle = HoodConstants.HOOD_TARGET_POS_POLLEN_FAR;

                if (systemState != SystemState.READY_TO_SHOOT_POLLEN_FAR)
                    systemState = SystemState.PREPARING_POLLEN_SHOOT_FAR;

                break;

            case SHOOT_POLLEN_AUTO:

                targetFlywheelSpeed = parametersCalculator.getFlywheelSpeedForPollen(new Pose2d()); //TODO for now, we give a target pos to the shooterParametor function, but later, the shooterParametererCalculator class itself will contains the target pos for turret, flywheel and hood
                targetHoodAngle = parametersCalculator.getHoodAngleForPollen(new Pose2d());//same

                if (systemState != SystemState.READY_TO_SHOOT_POLLEN_AUTO)
                    systemState = SystemState.PREPARING_POLLEN_SHOOT_AUTO;

                break;

            case SHOOT_NECTAR_NEAR:

                targetFlywheelSpeed = FlywheelConstants.FLYWHEEL_TARGET_SPEED_NECTAR_NEAR;
                targetHoodAngle = HoodConstants.HOOD_TARGET_POS_NECTAR_NEAR;

                if (systemState != SystemState.READY_TO_SHOOT_NECTAR_NEAR)
                    systemState = SystemState.PREPARING_NECTAR_SHOOT_NEAR;

                break;

            case SHOOT_NECTAR_MID:

                targetFlywheelSpeed = FlywheelConstants.FLYWHEEL_TARGET_SPEED_NECTAR_MID;
                targetHoodAngle = HoodConstants.HOOD_TARGET_POS_NECTAR_MID;

                if (systemState != SystemState.READY_TO_SHOOT_NECTAR_MID)
                    systemState = SystemState.PREPARING_NECTAR_SHOOT_MID;

                break;

            case SHOOT_NECTAR_FAR:

                targetFlywheelSpeed = FlywheelConstants.FLYWHEEL_TARGET_SPEED_NECTAR_FAR;
                targetHoodAngle = HoodConstants.HOOD_TARGET_POS_NECTAR_FAR;

                if (systemState != SystemState.READY_TO_SHOOT_NECTAR_FAR)
                    systemState = SystemState.PREPARING_NECTAR_SHOOT_FAR;

                break;

            case SHOOT_NECTAR_AUTO:

                targetFlywheelSpeed = parametersCalculator.getFlywheelSpeedForNectar(new Pose2d()); //same
                targetHoodAngle = parametersCalculator.getHoodAngleForNectar(new Pose2d()); //same

                if (systemState != SystemState.READY_TO_SHOOT_NECTAR_AUTO)
                    systemState = SystemState.PREPARING_NECTAR_SHOOT_AUTO;

                break;
            default:
                telemetry.addLine("Please enter a valid wanted state");
        }

        boolean isAtTargetSpeed = UtilsRBL.isInRange(flywheelIO.getFlywheelVelocity(), targetFlywheelSpeed, FlywheelConstants.FLYWHEEL_TOLERANCE);

        switch (systemState)
        {
            case IDLE:
            case STOP:
                break;

            case PREPARING_POLLEN_SHOOT_NEAR:

                if (isAtTargetSpeed)
                    systemState = SystemState.READY_TO_SHOOT_POLLEN_NEAR;

                break;

            case PREPARING_POLLEN_SHOOT_MID:

                if (isAtTargetSpeed)
                    systemState = SystemState.READY_TO_SHOOT_POLLEN_MID;

                break;

            case PREPARING_POLLEN_SHOOT_FAR:

                if (isAtTargetSpeed)
                    systemState = SystemState.READY_TO_SHOOT_POLLEN_FAR;

                break;

            case PREPARING_POLLEN_SHOOT_AUTO:

                if (isAtTargetSpeed)
                    systemState = SystemState.READY_TO_SHOOT_POLLEN_AUTO;

                break;

            case PREPARING_NECTAR_SHOOT_NEAR:

                if (isAtTargetSpeed)
                    systemState = SystemState.READY_TO_SHOOT_NECTAR_NEAR;

                break;

            case PREPARING_NECTAR_SHOOT_MID:

                if (isAtTargetSpeed)
                    systemState = SystemState.READY_TO_SHOOT_NECTAR_MID;

                break;

            case PREPARING_NECTAR_SHOOT_FAR:

                if (isAtTargetSpeed)
                    systemState = SystemState.READY_TO_SHOOT_NECTAR_FAR;

                break;

            case PREPARING_NECTAR_SHOOT_AUTO:

                if (isAtTargetSpeed)
                    systemState = SystemState.READY_TO_SHOOT_NECTAR_AUTO;

                break;

            case READY_TO_SHOOT_POLLEN_NEAR:
            case READY_TO_SHOOT_POLLEN_MID:
            case READY_TO_SHOOT_POLLEN_FAR:
            case READY_TO_SHOOT_POLLEN_AUTO:
            case READY_TO_SHOOT_NECTAR_NEAR:
            case READY_TO_SHOOT_NECTAR_MID:
            case READY_TO_SHOOT_NECTAR_FAR:
            case READY_TO_SHOOT_NECTAR_AUTO:
                break;
            default:
                telemetry.addLine("System state isn't valid ");
        }
    }
}
