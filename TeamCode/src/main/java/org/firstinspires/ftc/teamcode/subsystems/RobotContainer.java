package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.MecanumLocalizer;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.LyonLib.drive.DriveIO;
import org.firstinspires.ftc.LyonLib.kinematics.MecanumKinematics;
import org.firstinspires.ftc.LyonLib.localization.RobotLocalizer;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;
import org.firstinspires.ftc.LyonLib.logging.WpilogFileLogger;
import org.firstinspires.ftc.LyonLib.drive.MecanumDriveIO;

import org.firstinspires.ftc.teamcode.commands.ChangeHeadingTargetCmd;
import org.firstinspires.ftc.teamcode.commands.ChangeTurretTarget;
import org.firstinspires.ftc.teamcode.commands.OverrideDrivetrainInputsCmd;
import org.firstinspires.ftc.teamcode.subsystems.ShooterParametersCalculator.ShooterParametersCalculator;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DcMotorWheelIO;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.OTOS;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryWithPinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;


public class RobotContainer{
    public enum alliance
    {
        RED,
        BLUE
    }
    boolean inAuto = true;
    WpilogFileLogger logger = WpilogFileLogger.inDirectory("/sdcard/FIRST/RBLLogs");
    public RobotContainer(HardwareMap hmap, Telemetry telemetry, GamepadEx gamepad1, GamepadEx gamepad2)
    {
        drivetrain = new DrivetrainSubsystem(getDrivetrainIO(hmap), getLocalizer(logger, hmap), logger ,telemetry);

        shooterParametersCalculator = new ShooterParametersCalculator(()-> drivetrain.getCurrentPose(), telemetry);
        turret = new TurretSubsystem(hmap, telemetry, shooterParametersCalculator);
        shooter = new ShooterSubsystem(hmap, telemetry, shooterParametersCalculator);

        CommandScheduler.getInstance().registerSubsystem(turret, shooter);
        BindCommands(gamepad1,gamepad2);
    }

    public RobotContainer(HardwareMap hmap, Telemetry telemetry)
    {
        drivetrain = new DrivetrainSubsystem(getDrivetrainIO(hmap), getLocalizer(logger, hmap), logger ,telemetry);
    }

    public void init()
    {
        if (!inAuto)
        {
            drivetrain.setWantedMode(DrivetrainSubsystem.DriveMode.FRICTION_NEGLECTED);
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

    public void BindCommands(GamepadEx gamepad1, GamepadEx gamepad2)
    {
        //Gamepad 1
        drivetrain.configureManualSuppliers(
                () -> -gamepad1.getLeftY(),
                () -> -gamepad1.getLeftX(),
                () -> -gamepad1.getRightX()); //+ gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        Button PI2_CCW = new GamepadButton(gamepad1,Constants.PI2_CCW_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, -Math.PI/2.0, true));

        Button PI2_CW = new GamepadButton(gamepad1,Constants.PI2_CW_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, Math.PI/2.0, true));

        Button PI = new GamepadButton(gamepad1,Constants.PI_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, Math.PI, true));

        Button ZERO_HEADING = new GamepadButton(gamepad1,Constants.ZERO_HEADING_BUTTON)
                .whenPressed(new ChangeHeadingTargetCmd(drivetrain, 0.0, false));

        Button RIGHT_DASH = new GamepadButton(gamepad1, Constants.RIGHT_DASH_BUTTON)
                .whileHeld(new OverrideDrivetrainInputsCmd(drivetrain,0.0,-1.0,0.0),
                           gamepad1.getButton(Constants.LEFT_DASH_BUTTON));

        Button LEFT_DASH = new GamepadButton(gamepad1, Constants.LEFT_DASH_BUTTON)
                .whileHeld(new OverrideDrivetrainInputsCmd(drivetrain,0.0,1.0,0.0),
                           gamepad1.getButton(Constants.RIGHT_DASH_BUTTON));

        Button TRACK_RED_PUBLIC_CELL = new GamepadButton(gamepad2, Constants.TRACK_RED_PUBLIC_CELL)
                .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.TRACK_BLUE_PUBLIC_CELL));

        Button TRACK_BLUE_OPPOSITE_CELL = new GamepadButton(gamepad2, Constants.TRACK_BLUE_OPPOSITE_CELL)
                .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.TRACK_BLUE_OPPOSITE_CELL));

        Button STOP_TURRET = new GamepadButton(gamepad2, Constants.STOP_TURRET)
                .whenPressed(new ChangeTurretTarget(turret, TurretSubsystem.WantedState.STAND_BY));

        inAuto = false;
    }

    // --- Subsystems ---

    private DrivetrainSubsystem drivetrain;
    private TurretSubsystem turret;
    private ShooterSubsystem shooter;
    private ShooterParametersCalculator shooterParametersCalculator;


    public DrivetrainSubsystem getDrivetrain(){return drivetrain;}
    public TurretSubsystem getTurret(){return turret;}
    public ShooterSubsystem getShooter(){return shooter;}

    // --- IO getters ---
    public DriveIO getDrivetrainIO(HardwareMap hmap )
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

    public RobotLocalizer getLocalizer(RealtimeLogger logger, HardwareMap hmap)
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

        localizer.Init(new Pose2d());

        return localizer;
    }
}
