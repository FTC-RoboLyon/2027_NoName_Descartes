package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.MecanumLocalizer;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.LyonLib.utils.GamepadExEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.LyonLib.drive.DriveIO;
import org.firstinspires.ftc.LyonLib.kinematics.MecanumKinematics;
import org.firstinspires.ftc.LyonLib.localization.RobotLocalizer;
import org.firstinspires.ftc.LyonLib.logging.WpilogFileLogger;
import org.firstinspires.ftc.LyonLib.drive.MecanumDriveIO;

import org.firstinspires.ftc.teamcode.commands.ChangeHeadingTargetCmd;
import org.firstinspires.ftc.teamcode.commands.OverrideDrivetrainInputsCmd;
import org.firstinspires.ftc.teamcode.subsystems.shooterParametersCalculator.ShooterParametersCalculator;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.camera.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.CRServoWheelIO;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.OTOS;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryWithPinpoint;


public class RobotContainer{
    public enum RobotMode
    {
        AUTO_RED,
        AUTO_BLUE,
        TELEOP_BLUE,
        TELEOP_RED
    }

    public enum GamePiece
    {
        POLLEN,
        RED_NECTAR,
        BLUE_NECTAR
    }

    private WpilogFileLogger logger = WpilogFileLogger.inDirectory("/sdcard/FIRST/RBLLogs");
    private RobotMode robotMode;
    private GamepadExEx AlexisGamepad, VictorGamepad;
    private Telemetry telemetry;

    public RobotContainer(Telemetry telemetry, GamepadExEx gamepad1, GamepadExEx gamepad2, RobotMode robotMode)
    {
        if (robotMode != RobotMode.TELEOP_BLUE && robotMode != RobotMode.TELEOP_RED)
        {
            throw new RuntimeException("Teleop RobotContainer constructor used with a auto RobotMode");
        }

        if (Constants.ROBOT_MODEL != Constants.RobotModel.COMPETITION)
        {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        this.robotMode = robotMode;

        AlexisGamepad = gamepad1;
        VictorGamepad = gamepad2;
        this.telemetry = telemetry;
    }

    public RobotContainer(Telemetry telemetry, RobotMode robotMode)
    {
        if (robotMode == RobotMode.TELEOP_BLUE || robotMode == RobotMode.TELEOP_RED)
        {
            throw new RuntimeException("Auto RobotContainer constructor used with a Teleop RobotMode");
        }

        if (Constants.ROBOT_MODEL != Constants.RobotModel.COMPETITION)
        {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }

        this.robotMode = robotMode;
        this.telemetry = telemetry;
    }

    public void initializeSubsystems(HardwareMap hmap)
    {
        drivetrain = new DrivetrainSubsystem(getDrivetrainIO(hmap), getLocalizer(hmap), new Pose2d(),logger ,telemetry);
        intake = new IntakeSubsystem(new IntakeIO(hmap));
        camera = new CameraSubsystem(hmap.get(Limelight3A.class, "cam_ille"), this, logger, shootParams);

        shooterParametersCalculator = new ShooterParametersCalculator( ()-> drivetrain.getCurrentPose(), telemetry);
        shooterSubsystem = new ShooterSubsystem(new HoodIO(hmap), new FlywheelIO(hmap), shooterParametersCalculator);

        //turret = new TurretSubsystem(hmap, telemetry, shooterParametersCalculator);

        voltageCompensator = new VoltageCompensator(hmap);

        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem, frontIntake, backIntake);
        this.robotMode = robotMode;
        this.telemetry = telemetry;
    }

    // --- Command Based Structure ---
    public void init()
    {
        if (robotMode == RobotMode.TELEOP_BLUE || robotMode == RobotMode.TELEOP_RED)
        {
            drivetrain.setWantedMode(DrivetrainSubsystem.DriveMode.FRICTION_NEGLECTED);

            intake.SetControlMode(ControlMode.MANUAL_VOLTAGE);

            shooterSubsystem.setFlywheelControlMode(ControlMode.MANUAL_VELOCITY);
            shooterSubsystem.setHoodControlMode(ControlMode.DISABLED);
            shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.STAND_BY);
            //turret.setWantedState(TurretSubsystem.WantedState.STAND_BY);


            BindCommands();
        }
        else
        {
            //TODO : choose for auto;
        }
    }

    public void start()
    {
        drivetrain.start();
        //turret.resetTurret();
    }

    public void periodic()
    {
        CommandScheduler.getInstance().run();
        voltageCompensator.update();
        logger.flush();
    }

    public void end()
    {
        //turret.setWantedState(TurretSubsystem.WantedState.STAND_BY);
        intake.SetControlMode(ControlMode.DISABLED);
        drivetrain.setWantedMode(DrivetrainSubsystem.DriveMode.DISABLE);


        resetCommands();
        logger.close();
    }

    public void resetCommands()
    {
        CommandScheduler.getInstance().reset();
    }

    // --- Public Methods ---
    public Pose2d getRobotPose()
    {
        return drivetrain.getCurrentPose();
    }

    // --- Private methods ---
    private void BindCommands()
    {
        //Gamepad 1
        drivetrain.configureManualSuppliers(
                () -> -AlexisGamepad.getLeftY(),
                () -> -AlexisGamepad.getLeftX(),
                () -> -AlexisGamepad.getRightX() + AlexisGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - AlexisGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        intake.ConfigureManualSupplier(()-> VictorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        shooterSubsystem.setFlywheelManualVoltageSupplier(() -> VictorGamepad.getLeftY());
        shooterSubsystem.setSuppliersManualSpeedSuppliers(
                ()-> VictorGamepad.getButton(GamepadKeys.Button.CROSS),
                ()-> VictorGamepad.getButton(GamepadKeys.Button.CIRCLE),
                ()-> VictorGamepad.getButton(GamepadKeys.Button.TRIANGLE),
                ()-> VictorGamepad.getButton(GamepadKeys.Button.SQUARE));

        Button PI2_CCW = new GamepadButton(AlexisGamepad,Constants.PI2_CCW_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, Math.PI/2.0, true));

        Button PI2_CW = new GamepadButton(AlexisGamepad,Constants.PI2_CW_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, -Math.PI/2.0, true));

        Button PI = new GamepadButton(AlexisGamepad,Constants.PI_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, Math.PI, true));

        Button ZERO_HEADING = new GamepadButton(AlexisGamepad,Constants.ZERO_HEADING_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, 0.0, false));

        Button RIGHT_DASH = new GamepadButton(AlexisGamepad, Constants.RIGHT_DASH_BUTTON)
                .whenPressed(new OverrideDrivetrainInputsCmd(drivetrain,0.0,-1.0,0.0)
                .interruptOn(()->(AlexisGamepad.wasJustPressed(Constants.LEFT_DASH_BUTTON) || !AlexisGamepad.getButton(Constants.RIGHT_DASH_BUTTON))));

        Button LEFT_DASH = new GamepadButton(AlexisGamepad, Constants.LEFT_DASH_BUTTON)
                .whenPressed(new OverrideDrivetrainInputsCmd(drivetrain,0.0,1.0,0.0)
                .interruptOn(()->(AlexisGamepad.wasJustPressed(Constants.RIGHT_DASH_BUTTON) || !AlexisGamepad.getButton(Constants.LEFT_DASH_BUTTON))));

        //Button TRACK_RED_PUBLIC_CELL = new GamepadButton(VictorGamepad, Constants.TRACK_RED_PUBLIC_CELL)
        //        .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.TRACK_BLUE_PUBLIC_CELL));
//
        //Button TRACK_BLUE_OPPOSITE_CELL = new GamepadButton(VictorGamepad, Constants.TRACK_BLUE_OPPOSITE_CELL)
        //        .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.TRACK_BLUE_OPPOSITE_CELL));
//
        //Button STOP_TURRET = new GamepadButton(VictorGamepad, Constants.STOP_TURRET)
        //        .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.STAND_BY));
    }

    // --- Subsystems ---

    public static DrivetrainSubsystem drivetrain;
    public static IntakeSubsystem intake;
    public static ShooterParametersCalculator shooterParametersCalculator;
    public static ShooterSubsystem shooterSubsystem;

    //public static TurretSubsystem turret;
    public static VoltageCompensator voltageCompensator;
    public static CameraSubsystem camera;

    // --- IO getters ---
    private DriveIO getDrivetrainIO(HardwareMap hmap)
    {
        MecanumKinematics kinematics = new MecanumKinematics(DrivetrainConstants.TRACK_WIDTH_METERS,
                                                            DrivetrainConstants.WHEEL_BASE_METERS);

        return new MecanumDriveIO(
                new CRServoWheelIO(hmap, DrivetrainConstants.FRONT_LEFT_NAME, DrivetrainConstants.FRONT_LEFT_REVERSED),
                new CRServoWheelIO(hmap, DrivetrainConstants.FRONT_RIGHT_NAME, DrivetrainConstants.FRONT_RIGHT_REVERSED),
                new CRServoWheelIO(hmap, DrivetrainConstants.REAR_LEFT_NAME, DrivetrainConstants.REAR_LEFT_REVERSED),
                new CRServoWheelIO(hmap, DrivetrainConstants.REAR_RIGHT_NAME, DrivetrainConstants.REAR_RIGHT_REVERSED),
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
