package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.LyonLib.drive.DriveIO;
import org.firstinspires.ftc.LyonLib.drive.MecanumDriveIO;
import org.firstinspires.ftc.LyonLib.kinematics.MecanumKinematics;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.MecanumLocalizer;
import org.firstinspires.ftc.LyonLib.localization.RobotLocalizer;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;
import org.firstinspires.ftc.LyonLib.logging.WpilogFileLogger;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DcMotorWheelIO;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.OTOS;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryWithPinpoint;
@Config
public class miniRobot {

    private TurretSubsystem turret;
    private Telemetry telemetry;
    private VoltageCompensator voltageCompensator;
    private Pose2d posRobot = new Pose2d();
    public static double x = 2.11, y = 2.5, head = 0.0;


    private DrivetrainSubsystem drive;

    public enum alliance{
        BLUE,
        RED
    }
    private alliance alliance;
    public alliance getAlliance(){return alliance;}

    public miniRobot (HardwareMap hmap, Telemetry telemetry, alliance alliance1, Gamepad gamepad){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry =  new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //DriveIO io = getDriveIO(hmap);

        //WpilogFileLogger logger = WpilogFileLogger.inDirectory("/sdcard/FIRST/java/logs");
        //VoltageCompensator vc = new VoltageCompensator(hmap); //not implemented yet

        //RobotLocalizer localizer = getLocalizer(logger, hmap);

        //drive = new DrivetrainSubsystem(io, localizer, logger, telemetry);
        //drive.configureManualSuppliers(
        //        () -> -gamepad.left_stick_y,   // avant
        //        () -> -gamepad.left_stick_x,   // strafe gauche
        //        () -> -gamepad.right_stick_x); // rotation CCW

        //localizer.Init(new Pose2d(0.16, 0.22, 0.0));
        alliance = alliance1;
        this.telemetry = telemetry;

        turret = new TurretSubsystem(hmap, this, ()-> posRobot);

        voltageCompensator = new VoltageCompensator(hmap);
        CommandScheduler.getInstance().registerSubsystem(turret);

        GamepadEx gamepadEx = new GamepadEx(gamepad);


        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> turret.setWantedState(TurretSubsystem.WantedState.TRACK_RED_PUBLIC_CELL)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(() -> turret.setWantedState(TurretSubsystem.WantedState.TRACK_BLUE_OPPOSITE_CELL)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> turret.setWantedState(TurretSubsystem.WantedState.STAND_BY)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> turret.setWantedState(TurretSubsystem.WantedState.TRACK_BLUE_PUBLIC_CELL)));
    }
    public DrivetrainSubsystem getDrive(){return drive;}

    public void update(){
        voltageCompensator.update();
        posRobot = new Pose2d(x, y, head);
        turret.getPidTurret().setGains(TurretConstants.turretKP, TurretConstants.turretKI, TurretConstants.turretKD);
        telemetry.addData("position", posRobot);
        //drive.periodic();
        //telemetry.addData("position",drive.getCurrentPose());
        //double[] p = drive.getCurrentPose().toArray(); // [x, y, headingRad] in metres
        //telemetry.addData("x (m)", "%.3f", p[0]);
        //telemetry.addData("y (m)", "%.3f", p[1]);
        //telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(p[2]));
        //telemetry.addData("robo pos", drive.getCurrentPose());
        //telemetry.addData("voltage registred", VoltageCompensator.getVoltage());
    }

    public void start()
    {
        //drive.setWantedMode(DrivetrainSubsystem.DriveMode.ROBOT_CENTRIC);
        //drive.start();
        //drive.resetPose(new Pose2d(0.16, 0.22, 0.0));
    }


    public Telemetry getTelemetry(){return telemetry;}

    public Pose2d getRobotPos(){return drive.getCurrentPose();}
    public TurretSubsystem getTurret(){return turret;}

    @NonNull
    private DriveIO getDriveIO(HardwareMap hardwareMap) {
        MecanumKinematics kinematics = new MecanumKinematics(
                DrivetrainConstants.TRACK_WIDTH_METERS, DrivetrainConstants.WHEEL_BASE_METERS);

        return new MecanumDriveIO(
                new DcMotorWheelIO(hardwareMap, DrivetrainConstants.FRONT_LEFT_NAME, DrivetrainConstants.FRONT_LEFT_REVERSED),
                new DcMotorWheelIO(hardwareMap, DrivetrainConstants.FRONT_RIGHT_NAME, DrivetrainConstants.FRONT_RIGHT_REVERSED),
                new DcMotorWheelIO(hardwareMap, DrivetrainConstants.REAR_LEFT_NAME, DrivetrainConstants.REAR_LEFT_REVERSED),
                new DcMotorWheelIO(hardwareMap, DrivetrainConstants.REAR_RIGHT_NAME, DrivetrainConstants.REAR_RIGHT_REVERSED),
                kinematics, DrivetrainConstants.MAX_WHEEL_SPEED_MPS);
    }

    @NonNull
    private RobotLocalizer getLocalizer(RealtimeLogger logger, HardwareMap hardwareMap) {
        // Pod offsets / calibration from the localization team (LocalizationPrinter).
        OdometryWithPinpoint.OdometryPod frontPod = new OdometryWithPinpoint.OdometryPod(
                DrivetrainConstants.POD1_OFFSETS.xMeters, DrivetrainConstants.POD1_OFFSETS.yMeters, DrivetrainConstants.POD1_OFFSETS.headingRadians,
                DrivetrainConstants.POD1_WHEEL_RADIUS, DrivetrainConstants.POD1_ENCODER_RESOLUTION, DrivetrainConstants.POD1_ENCODER_REVERSED);

        OdometryWithPinpoint.OdometryPod strafePod = new OdometryWithPinpoint.OdometryPod(
                DrivetrainConstants.POD2_OFFSETS.xMeters, DrivetrainConstants.POD2_OFFSETS.yMeters, DrivetrainConstants.POD2_OFFSETS.headingRadians,
                DrivetrainConstants.POD2_WHEEL_RADIUS, DrivetrainConstants.POD2_ENCODER_RESOLUTION, DrivetrainConstants.POD2_ENCODER_REVERSED);

        OdometryWithPinpoint odo = new OdometryWithPinpoint(
                hardwareMap, "odoPinpoint", frontPod, strafePod, new Pose2d(0.01, 0.01, 1e-3),0.9997);

        OTOS otos = new OTOS(hardwareMap, "otos", 0.0, 0.0, 0.0, new Pose2d(0.0,0.0,0.0),0.995);

        //NavX navx = new NavX(hardwareMap,"navx", new Pose2d(1e14, 1e14, 0.1));

        MecanumLocalizer localizer = new MecanumLocalizer(logger);
        localizer.AddSensor(odo);
        localizer.AddSensor(otos);
        //localizer.AddSensor(navx);

        return localizer;
    }
}

