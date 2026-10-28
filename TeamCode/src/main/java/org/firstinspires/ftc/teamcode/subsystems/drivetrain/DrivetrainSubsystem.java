package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.drive.DriveIO;
import org.firstinspires.ftc.LyonLib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.RobotLocalizer;
import org.firstinspires.ftc.LyonLib.logging.NoOpLogger;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;
import org.firstinspires.ftc.LyonLib.trajectory.HolonomicTrajectory;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

/**
 * Robot-agnostic drivetrain subsystem. Talks only in ChassisSpeeds / Pose2d, so
 * it is identical for mecanum or swerve and for FRC or FTC.
 *
 * <p>Pose source: if a {@link RobotLocalizer} is supplied, it is the authoritative
 * field pose (fused dead-wheel odometry + OTOS + future AprilTag), and DriveIO's
 * wheel odometry is kept only as a diagnostic. With no Localizer, the subsystem
 * falls back to DriveIO wheel odometry.
 *
 * <p>A DriveMode state machine (wanted -> system) selects the behaviour. This
 * subsystem does NOT flush the logger — the OpMode flushes once per loop.
 */
public final class DrivetrainSubsystem extends SubsystemBase
{

    public enum DriveMode
    {
        PATH_FOLLOWING,   // follow a pre-generated HolonomicTrajectory (feedforward + PID)
        GO_TO_POSE,       // drive straight to a target pose with a PID
        ROBOT_CENTRIC,    // manual, axes are relative to the robot
        FIELD_CENTRIC,    // manual, axes are relative to the field
        FRICTION_NEGLECTED, //manual, axes are relative to the field, heading is defined by a setpoint and always corrected
        DISABLE           // hold still
    }


    private final DriveIO io;
    private final DriveIO.DriveIOInputs inputs = new DriveIO.DriveIOInputs();
    private final RealtimeLogger logger;
    private final RobotLocalizer localizer;
    private final Telemetry telemetry;

    private double now;

    private DriveMode wantedMode = DriveMode.DISABLE;
    private DriveMode systemMode = DriveMode.DISABLE;

    private ChassisSpeeds lastCommanded = new ChassisSpeeds();
    private ChassisSpeeds overridenInputs;

    // Manual axis suppliers, in [-1, 1]. Default to neutral.
    private DoubleSupplier forwardSupplier = () -> 0.0;
    private DoubleSupplier leftSupplier = () -> 0.0;
    private DoubleSupplier ccwSupplier = () -> 0.0;


    // GO_TO_POSE: PID on distance-to-target, and a continuous PID on heading.
    private final PidRBL distanceController = new PidRBL(
            DrivetrainConstants.DISTANCE_KP,
            DrivetrainConstants.DISTANCE_KI,
            DrivetrainConstants.DISTANCE_KD);
    private final PidRBL headingController = new PidRBL(
            DrivetrainConstants.HEADING_KP,
            DrivetrainConstants.HEADING_KI,
            DrivetrainConstants.HEADING_KD);

    // PATH_FOLLOWING: per-axis position-error PID (heading reuses the heading controller).
    private final PidRBL pathXController = new PidRBL(DrivetrainConstants.PATH_X_KP, 0.0, 0.0);
    private final PidRBL pathYController = new PidRBL(DrivetrainConstants.PATH_Y_KP, 0.0, 0.0);


    // Target for GO_TO_POSE.
    private Pose2d targetPose = new Pose2d();

    // Target for FRICTION_NEGLECTED (also used with inputs override)
    private double targetHeading; //in rad

    // Active trajectory for PATH_FOLLOWING.
    private HolonomicTrajectory activePath = null;
    private double pathStartTimeSeconds = 0.0;

    private boolean previousInputsOverriden = false;

    /** no Logging + fused pose from the RobotLocalizer (recommended). */
    public DrivetrainSubsystem(DriveIO io, RobotLocalizer localizer, Pose2d InitPos, Telemetry telemetry) {
        this(io, localizer, InitPos, new NoOpLogger(), telemetry);
    }

    /** Logging + fused pose from the RobotLocalizer (recommended). */
    public DrivetrainSubsystem(DriveIO io, RobotLocalizer localizer, Pose2d InitPos, RealtimeLogger logger, Telemetry telemetry)
    {
        this.io = io;
        this.localizer = localizer;
        this.logger = logger;
        this.telemetry = telemetry;

        localizer.SetDrivetrainParameters(io.getDrivetrainKinematics(), io.getWheelInputs(), DrivetrainConstants.MODEL_PROCESS_NOISE);
        localizer.Init(InitPos);

        // The distance PID output is a speed magnitude in m/s.
        distanceController.setOutputLimits(
                -DrivetrainConstants.MAX_LINEAR_SPEED_MPS,
                DrivetrainConstants.MAX_LINEAR_SPEED_MPS);

        // Heading wraps around: continuous over [-pi, pi], output is omega in rad/s.
        headingController.setContinuous(true);
        headingController.setInputLimits(-Math.PI, Math.PI);
        headingController.setOutputLimits(
                -DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
                DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC);

        // Path position correctors output a velocity correction, bounded to the speed limit.
        pathXController.setOutputLimits(
                -DrivetrainConstants.MAX_LINEAR_SPEED_MPS, DrivetrainConstants.MAX_LINEAR_SPEED_MPS);
        pathYController.setOutputLimits(
                -DrivetrainConstants.MAX_LINEAR_SPEED_MPS, DrivetrainConstants.MAX_LINEAR_SPEED_MPS);
    }

    // ---- Configuration ----------------------------------------------------

    /** Wires the manual driving axes (read every loop in ROBOT_CENTRIC / FIELD_CENTRIC). */
    public void configureManualSuppliers(DoubleSupplier forward, DoubleSupplier left, DoubleSupplier ccw)
    {
        this.forwardSupplier = forward;
        this.leftSupplier = left;
        this.ccwSupplier = ccw;
    }

    public void setWantedMode(DriveMode mode)
    {
        this.wantedMode = mode;
    }

    public DriveMode getSystemMode()
    {
        return systemMode;
    }

    /** Target pose for GO_TO_POSE. Switch the wanted mode separately. */
    public void setTargetPose(Pose2d target)
    {
        this.targetPose = target;
    }

    /** Loads the trajectory to follow. Switch the wanted mode to PATH_FOLLOWING separately. */
    public void setPath(HolonomicTrajectory path)
    {
        this.activePath = path;
    }

    /** Set a heading target for FRICTION_NEGLECTED. Switch the wanted mode to FRICTION_NEGELECTED separately. */
    public void setTargetHeading(double targetHeading)
    {
        this.targetHeading = targetHeading;
    }

    /** Add a given angle value in rad (can be negative) to the current heading target of FRICTION_NEGELECTED DriveMode.
     *  Switch the wanted mode to FRICTION_NEGELECTED separately. */
    public void incrementHeadingTarget(double value)
    {
        targetHeading = UtilsRBL.wrapNegPiToPi(targetHeading + value);
    }

    /** Latest authoritative (fused) field pose. */
    public Pose2d getCurrentPose()
    {
        return inputs.currentPose;
    }

    public void resetPose(Pose2d newPose)
    {
        io.resetPose(newPose);
        localizer.ResetPos(newPose);
    }

    public void start()
    {
        localizer.StartLocalizer(System.nanoTime());
    }

    public void stop()
    {
        setWantedMode(DriveMode.DISABLE);
    }

    /** True once the active trajectory's total time has elapsed. */
    public boolean isPathFinished()
    {
        if (activePath == null || activePath.isEmpty())
            return true;
        return (System.nanoTime() * 1e-9 - pathStartTimeSeconds) >= activePath.getTotalTimeSeconds();
    }

    public void UpdatePIDCoefs() //useful for tunning
    {
        distanceController.setGains(DrivetrainConstants.DISTANCE_KP,
                DrivetrainConstants.DISTANCE_KI,
                DrivetrainConstants.DISTANCE_KD);

        headingController.setGains(DrivetrainConstants.HEADING_KP,
                DrivetrainConstants.HEADING_KI,
                DrivetrainConstants.HEADING_KD);

        pathXController.setGains(DrivetrainConstants.PATH_X_KP, 0.0, 0.0);
        pathYController.setGains(DrivetrainConstants.PATH_Y_KP, 0.0, 0.0);
    }

    /** Use this function to override all other inputs. Params must be robot centric. If omega power is null the robot will maintain heading with a PID controller.*/
    public void OverrideInputs(double forward, double strafe, double omega)
    {
        overridenInputs = new ChassisSpeeds(forward,strafe,omega);

        if (!previousInputsOverriden)
        {
            targetHeading = UtilsRBL.wrapNegPiToPi(inputs.currentPose.headingRadians);
        }

        previousInputsOverriden = true;
    }


    public void periodic()
    {
        io.updateInputs(inputs);

        now = System.nanoTime() * 1e-9;

        inputs.currentPose = localizer.Update(now);

        handleStateTransition();

        ChassisSpeeds output;

        if (overridenInputs == null)
        {
            output = computeOutput();
            previousInputsOverriden = false;
        }
        else
        {
            output = overridenInputs.copy();
            if(output.omega == 0.0)
            {
                output.omega = headingController.calculateWithRealTime(targetHeading, UtilsRBL.wrapNegPiToPi(inputs.currentPose.headingRadians), now);
            }
            overridenInputs = null;
        }

        io.applyChassisSpeeds(output);
        lastCommanded = output;

        logger.log("Drivetrain/WantedMode", wantedMode.name());
        logger.log("Drivetrain/SystemMode", systemMode.name());

        DriveTelemetry.logCommanded(logger, lastCommanded);
        DriveTelemetry.logInputs(logger, inputs);

        telemetry.addData("Target",targetHeading);
        telemetry.addData("Heading", UtilsRBL.wrapNegPiToPi(inputs.currentPose.headingRadians));
        telemetry.addData("InputsOverrriden", previousInputsOverriden);
    }


    /** Promotes wantedMode -> systemMode, running one-shot entry actions on change. */
    private void handleStateTransition()
    {
        if (wantedMode == systemMode)
            return;

        switch (wantedMode)
        {
            case PATH_FOLLOWING:
            {
                pathStartTimeSeconds = now;
                pathXController.reset(inputs.currentPose.xMeters);
                pathYController.reset(inputs.currentPose.yMeters);
                headingController.reset(inputs.currentPose.headingRadians);
                break;
            }
            case GO_TO_POSE:
            {
                // Reset the controllers so there's no derivative kick / stale integral.
                double distance = Math.hypot(targetPose.xMeters - inputs.currentPose.xMeters,
                        targetPose.yMeters - inputs.currentPose.yMeters);
                distanceController.reset(distance);
                headingController.reset(inputs.currentPose.headingRadians);
                break;
            }

            case FRICTION_NEGLECTED:
            {
                targetHeading = UtilsRBL.wrapNegPiToPi(inputs.currentPose.headingRadians);
                headingController.reset(targetHeading);
                break;
            }

            case ROBOT_CENTRIC:
            case FIELD_CENTRIC:
            case DISABLE:
            default:
                break;
        }
        systemMode = wantedMode;
    }

    /** Computes robot-relative speeds for the active mode. */
    private ChassisSpeeds computeOutput()
    {
        switch (systemMode)
        {
            case PATH_FOLLOWING:
                return followPath();
            case GO_TO_POSE:
                return goToPose();
            case ROBOT_CENTRIC:
                return robotCentric();
            case FIELD_CENTRIC:
                return fieldCentric();
            case FRICTION_NEGLECTED:
                return frictionNeglected();
            case DISABLE:
            default:
                return new ChassisSpeeds();
        }
    }

    // ---- Per-mode behaviours ---------------------------------------------

    private ChassisSpeeds robotCentric()
    {
        return new ChassisSpeeds(
                shape(forwardSupplier.getAsDouble()) * DrivetrainConstants.MAX_LINEAR_SPEED_MPS,
                shape(leftSupplier.getAsDouble()) * DrivetrainConstants.MAX_LINEAR_SPEED_MPS,
                shape(ccwSupplier.getAsDouble()) * DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC);
    }

    private ChassisSpeeds fieldCentric()
    {
        double vxField = shape(forwardSupplier.getAsDouble()) * DrivetrainConstants.MAX_LINEAR_SPEED_MPS;
        double vyField = shape(leftSupplier.getAsDouble()) * DrivetrainConstants.MAX_LINEAR_SPEED_MPS;
        double omega = shape(ccwSupplier.getAsDouble()) * DrivetrainConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC;
        return fieldToRobot(vxField, vyField, omega, inputs.currentPose.headingRadians);
    }

    private ChassisSpeeds goToPose()
    {
        double errorX = targetPose.xMeters - inputs.currentPose.xMeters;
        double errorY = targetPose.yMeters - inputs.currentPose.yMeters;
        double linearDistance = Math.hypot(errorX, errorY);

        // Static-friction compensation when not basically on top of the point.
        double frictionConstant = 0.0;
        if (linearDistance >= DrivetrainConstants.STATIC_FRICTION_DISTANCE_M) {
            frictionConstant = DrivetrainConstants.DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * DrivetrainConstants.MAX_LINEAR_SPEED_MPS;
        }

        // PID drives the distance to 0; magnitude only, the direction is geometric.
        double pidMagnitude = Math.abs(distanceController.calculateWithRealTime(0.0, linearDistance, now));
        double velocityOutput = Math.min(pidMagnitude + frictionConstant, DrivetrainConstants.MAX_LINEAR_SPEED_MPS);

        // Direction to the target, in the FIELD frame.
        double directionOfTravel = Math.atan2(errorY, errorX);
        double vxField = velocityOutput * Math.cos(directionOfTravel);
        double vyField = velocityOutput * Math.sin(directionOfTravel);

        // Heading handled by its own continuous PID.
        double omega = headingController.calculateWithRealTime(targetPose.headingRadians, UtilsRBL.wrapNegPiToPi(inputs.currentPose.headingRadians), now);

        logger.log("Drivetrain/DriveToPoint/linearDistance", linearDistance);
        logger.log("Drivetrain/DriveToPoint/velocityOutput", velocityOutput);
        logger.log("Drivetrain/DriveToPoint/directionOfTravelRad", directionOfTravel);

        return fieldToRobot(vxField, vyField, omega, inputs.currentPose.headingRadians);
    }

    /**
     * Holonomic trajectory follower: at the elapsed time, sample the desired pose
     * and field velocity (kinematic feedforward), add a per-axis PID correction on
     * the pose error, and rotate the result into the robot frame.
     */
    private ChassisSpeeds followPath()
    {
        if (activePath == null || activePath.isEmpty())
            return new ChassisSpeeds();

        double t = now - pathStartTimeSeconds;
        HolonomicTrajectory.Sample sample = activePath.sample(t);

        targetPose = sample.pose;



        // Field-relative: trajectory velocity feedforward + PID on the pose error.
        double vxField = sample.vx + pathXController.calculateWithRealTime(targetPose.xMeters, inputs.currentPose.xMeters, now);
        double vyField = sample.vy + pathYController.calculateWithRealTime(targetPose.yMeters, inputs.currentPose.yMeters, now);
        double omega = sample.omega + headingController.calculateWithRealTime(targetPose.headingRadians,
                UtilsRBL.wrapNegPiToPi(inputs.currentPose.headingRadians), now);

        logger.log("Drivetrain/PathFollow/t", t);
        logger.log("Drivetrain/PathFollow/desired", targetPose.toArray());
        logger.log("Drivetrain/PathFollow/errorX", targetPose.xMeters - inputs.currentPose.xMeters);
        logger.log("Drivetrain/PathFollow/errorY", targetPose.yMeters - inputs.currentPose.yMeters);

        return fieldToRobot(vxField, vyField, omega, inputs.currentPose.headingRadians);
    }

    private ChassisSpeeds frictionNeglected()
    {
        double vxField = shape(forwardSupplier.getAsDouble()) * DrivetrainConstants.MAX_LINEAR_SPEED_MPS;
        double vyField = shape(leftSupplier.getAsDouble()) * DrivetrainConstants.MAX_LINEAR_SPEED_MPS;
        targetHeading = UtilsRBL.wrapNegPiToPi(targetHeading + shape(ccwSupplier.getAsDouble())*DrivetrainConstants.MANUAL_SETPOINT);
        double omega = headingController.calculateWithRealTime(targetHeading, UtilsRBL.wrapNegPiToPi(inputs.currentPose.headingRadians), now);
        return fieldToRobot(vxField, vyField, omega, inputs.currentPose.headingRadians);
    }

    // ---- Helpers ----------------------------------------------------------

    /** Rotates a field-relative (vx, vy) into the robot frame for the given heading. */
    private static ChassisSpeeds fieldToRobot(double vxField, double vyField, double omega, double headingRad)
    {
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double vxRobot = cos * vxField + sin * vyField;
        double vyRobot = -sin * vxField + cos * vyField;
        return new ChassisSpeeds(vxRobot, vyRobot, omega);
    }

    private static double shape(double raw)
    {
        return UtilsRBL.squareKeepSign(UtilsRBL.applyDeadband(raw, DrivetrainConstants.JOYSTICK_DEADBAND));
    }
}
