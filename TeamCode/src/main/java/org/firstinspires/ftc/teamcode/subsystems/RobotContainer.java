package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.MecanumLocalizer;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.LyonLib.drive.DriveIO;
import org.firstinspires.ftc.LyonLib.kinematics.MecanumKinematics;
import org.firstinspires.ftc.LyonLib.localization.RobotLocalizer;
import org.firstinspires.ftc.LyonLib.logging.WpilogFileLogger;
import org.firstinspires.ftc.LyonLib.drive.MecanumDriveIO;

import org.firstinspires.ftc.teamcode.commands.ChangeHeadingTargetCmd;
import org.firstinspires.ftc.teamcode.commands.ChangeTurretTarget;
import org.firstinspires.ftc.teamcode.commands.OverrideDrivetrainInputsCmd;
import org.firstinspires.ftc.teamcode.subsystems.ShooterParametersCalculator.ShooterParametersCalculator;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DcMotorWheelIO;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.OTOS;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryWithPinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;


public class RobotContainer{
    public enum RobotMode
    {
        AUTO_RED,
        AUTO_BLUE,
        TELEOP_BLUE,
        TELEOP_RED
    }

    WpilogFileLogger logger = WpilogFileLogger.inDirectory("/sdcard/FIRST/RBLLogs");
    RobotMode robotMode;
    GamepadEx AlexisGamepad, VictorGamepad;

    public RobotContainer(HardwareMap hmap, Telemetry telemetry, GamepadEx gamepad1, GamepadEx gamepad2, RobotMode robotMode)
    {
        if (robotMode != RobotMode.TELEOP_BLUE && robotMode != RobotMode.TELEOP_RED)
        {
            throw new RuntimeException("Teleop RobotContainer constructor used with a auto RobotMode");
        }

        drivetrain = new DrivetrainSubsystem(getDrivetrainIO(hmap), getLocalizer(hmap), new Pose2d(),logger ,telemetry);
        frontIntake = new IntakeSubsystem(hmap, true);
        backIntake = new IntakeSubsystem(hmap, false);

        shooterParametersCalculator = new ShooterParametersCalculator(()-> drivetrain.getCurrentPose(), telemetry);
        turret = new TurretSubsystem(hmap, telemetry, shooterParametersCalculator);
        shooter = new ShooterSubsystem(hmap, telemetry, shooterParametersCalculator);

        voltageCompensator = new VoltageCompensator(hmap);

        CommandScheduler.getInstance().registerSubsystem(turret, shooter);

        this.robotMode = robotMode;

        AlexisGamepad = gamepad1;
        VictorGamepad = gamepad2;
    }

    public RobotContainer(HardwareMap hmap, Telemetry telemetry, RobotMode robotMode)
    {
        if (robotMode == RobotMode.TELEOP_BLUE || robotMode == RobotMode.TELEOP_RED)
        {
            throw new RuntimeException("Auto RobotContainer constructor used with a Teleop RobotMode");
        }

        drivetrain = new DrivetrainSubsystem(getDrivetrainIO(hmap), getLocalizer(hmap), new Pose2d(),logger ,telemetry); //TODO : add starting poses
        frontIntake = new IntakeSubsystem(hmap, true);
        backIntake = new IntakeSubsystem(hmap, false);
        voltageCompensator = new VoltageCompensator(hmap);

        this.robotMode = robotMode;
    }

    public void init()
    {
        if (robotMode == RobotMode.TELEOP_BLUE || robotMode == RobotMode.TELEOP_RED)
        {
            drivetrain.setWantedMode(DrivetrainSubsystem.DriveMode.FRICTION_NEGLECTED);
            frontIntake.SetControlMode(ControlMode.MANUAL_VOLTAGE);
            backIntake.SetControlMode(ControlMode.MANUAL_VOLTAGE);
            BindCommands();
        }
        else
        {
            //TODO : choose for auto;
        }

        turret.setWantedState(TurretSubsystem.WantedState.STAND_BY);
        shooter.setShooterControlMode(ControlMode.VELOCITY_DUTYCYCLE_PID_FF);
    }

    public void start()
    {
        drivetrain.start();
        shooter.start();
        turret.resetTurret();
    }

    public void periodic()
    {
        CommandScheduler.getInstance().run();
        voltageCompensator.update();
        shooterParametersCalculator.update();
        logger.flush();
    }

    public void end()
    {
        shooter.setShooterControlMode(ControlMode.DISABLED);
        resetCommands();
        logger.close();
    }

    public void resetCommands()
    {
        CommandScheduler.getInstance().reset();
    }

    // --- Private methods ---
    private void BindCommands()
    {
        //Gamepad 1
        drivetrain.configureManualSuppliers(
                () -> -AlexisGamepad.getLeftY(),
                () -> -AlexisGamepad.getLeftX(),
                () -> -AlexisGamepad.getRightX() + AlexisGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - AlexisGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        frontIntake.ConfigureManualSupplier(()-> VictorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        backIntake.ConfigureManualSupplier(()-> VictorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        Button PI2_CCW = new GamepadButton(AlexisGamepad,Constants.PI2_CCW_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, -Math.PI/2.0, true));

        Button PI2_CW = new GamepadButton(AlexisGamepad,Constants.PI2_CW_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, Math.PI/2.0, true));

        Button PI = new GamepadButton(AlexisGamepad,Constants.PI_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, Math.PI, true));

        Button ZERO_HEADING = new GamepadButton(AlexisGamepad,Constants.ZERO_HEADING_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, 0.0, false));

        Button RIGHT_DASH = new GamepadButton(AlexisGamepad, Constants.RIGHT_DASH_BUTTON)
                .whileHeld(new OverrideDrivetrainInputsCmd(drivetrain,0.0,-1.0,0.0),
                        AlexisGamepad.getButton(Constants.LEFT_DASH_BUTTON));

        Button LEFT_DASH = new GamepadButton(AlexisGamepad, Constants.LEFT_DASH_BUTTON)
                .whileHeld(new OverrideDrivetrainInputsCmd(drivetrain,0.0,1.0,0.0),
                        AlexisGamepad.getButton(Constants.RIGHT_DASH_BUTTON));

        Button TRACK_RED_PUBLIC_CELL = new GamepadButton(VictorGamepad, Constants.TRACK_RED_PUBLIC_CELL)
                .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.TRACK_BLUE_PUBLIC_CELL));

        Button TRACK_BLUE_OPPOSITE_CELL = new GamepadButton(VictorGamepad, Constants.TRACK_BLUE_OPPOSITE_CELL)
                .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.TRACK_BLUE_OPPOSITE_CELL));

        Button STOP_TURRET = new GamepadButton(VictorGamepad, Constants.STOP_TURRET)
                .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.STAND_BY));

    }

    // --- Subsystems ---

    public DrivetrainSubsystem drivetrain;
    public IntakeSubsystem frontIntake, backIntake;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;
    private ShooterParametersCalculator shooterParametersCalculator;
    public VoltageCompensator voltageCompensator;

    // --- IO getters ---
    private DriveIO getDrivetrainIO(HardwareMap hmap)
    {
        MecanumKinematics kinematics = new MecanumKinematics(DrivetrainConstants.TRACK_WIDTH_METERS,
                                                            DrivetrainConstants.WHEEL_BASE_METERS);

        return new MecanumDriveIO(
                new DcMotorWheelIO(hmap, DrivetrainConstants.FRONT_LEFT_NAME, DrivetrainConstants.FRONT_LEFT_REVERSED),
                new DcMotorWheelIO(hmap, DrivetrainConstants.FRONT_RIGHT_NAME, DrivetrainConstants.FRONT_RIGHT_REVERSED),
                new DcMotorWheelIO(hmap, DrivetrainConstants.REAR_LEFT_NAME, DrivetrainConstants.REAR_LEFT_REVERSED),
                new DcMotorWheelIO(hmap, DrivetrainConstants.REAR_RIGHT_NAME, DrivetrainConstants.REAR_RIGHT_REVERSED),
                kinematics, DrivetrainConstants.MAX_WHEEL_SPEED_MPS);
    }

    private RobotLocalizer getLocalizer(HardwareMap hmap)
    {
        OdometryWithPinpoint.OdometryPod frontPod = new OdometryWithPinpoint.OdometryPod(
                DrivetrainConstants.POD1_OFFSETS.xMeters, DrivetrainConstants.POD1_OFFSETS.yMeters, DrivetrainConstants.POD1_OFFSETS.headingRadians,
                DrivetrainConstants.POD1_WHEEL_RADIUS, DrivetrainConstants.POD1_ENCODER_RESOLUTION, DrivetrainConstants.POD1_ENCODER_REVERSED);

        OdometryWithPinpoint.OdometryPod strafePod = new OdometryWithPinpoint.OdometryPod(
                DrivetrainConstants.POD2_OFFSETS.xMeters, DrivetrainConstants.POD2_OFFSETS.yMeters, DrivetrainConstants.POD2_OFFSETS.headingRadians,
                DrivetrainConstants.POD2_WHEEL_RADIUS, DrivetrainConstants.POD2_ENCODER_RESOLUTION, DrivetrainConstants.POD2_ENCODER_REVERSED);

        OdometryWithPinpoint odo = new OdometryWithPinpoint(
                hmap, "odoPinpoint", frontPod, strafePod, new Pose2d(0.01, 0.01, 1e-3),0.9997);

        OTOS otos = new OTOS(hmap, DrivetrainConstants.OTOS_NAME,
                0.0, 0.0, 0.0, new Pose2d(0.0,0.0,0.0),0.995);

        MecanumLocalizer localizer = new MecanumLocalizer(logger);
        localizer.AddSensor(odo);
        localizer.AddSensor(otos);

        return localizer;
    }
}
