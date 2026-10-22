package org.firstinspires.ftc.teamcode.teleops;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.LyonLib.drive.DriveIO;
import org.firstinspires.ftc.LyonLib.drive.MecanumDriveIO;
import org.firstinspires.ftc.LyonLib.kinematics.MecanumKinematics;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.localization.MecanumLocalizer;
import org.firstinspires.ftc.LyonLib.localization.RobotLocalizer;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;
import org.firstinspires.ftc.LyonLib.logging.WpilogFileLogger;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
//import org.firstinspires.ftc.teamcode.GeneralTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DcMotorWheelIO;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.localization.NavX;
import org.firstinspires.ftc.teamcode.subsystems.localization.OTOS;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryWithPinpoint;

/**
 * Field-centric mecanum TeleOp. Wires four {@link DcMotorWheelIO} into a
 * {@link MecanumDriveIO}, and the localization stack (Pinpoint dead-wheel
 * odometry + OTOS, fused by {@link RobotLocalizer} into the platform-agnostic
 * {@link DrivetrainSubsystem}).
 * The subsystem drives field-centric using the fused pose, and everything is
 * logged to a WPILOG on the Control Hub for AdvantageScope.
 */
@TeleOp(name = "Mecanum TeleOp", group = "drive")
public class MecanumTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry =  new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        DriveIO io = getDriveIO();

        WpilogFileLogger logger = WpilogFileLogger.inDirectory("/sdcard/FIRST/java/logs");
        VoltageCompensator vc = new VoltageCompensator(hardwareMap); //not implemented yet

        RobotLocalizer localizer = getLocalizer(logger);

        DrivetrainSubsystem drive = new DrivetrainSubsystem(io, localizer, logger, telemetry);
        drive.configureManualSuppliers(
                () -> -gamepad1.left_stick_y,   // avant
                () -> -gamepad1.left_stick_x,   // strafe gauche
                () -> -gamepad1.right_stick_x); // rotation CCW

        // Seed the field origin AND initialise the localization sensors (this
        // calibrates the OTOS IMU, so it must happen during init, not mid-match).
        localizer.Init(new Pose2d(0.0,0.0,0.0));
        drive.resetPose(new Pose2d(0.0, 0.0, 0.0));

        telemetry.addLine("Mecanum ready - press PLAY");
        telemetry.update();
        waitForStart();

        drive.setWantedMode(DrivetrainSubsystem.DriveMode.FRICTION_NEGLECTED);
        drive.start();

        try {
            while (opModeIsActive()) {

                drive.periodic(); // updates the fused pose and drives the wheels

                //GeneralTelemetry.logMatch(logger, GeneralTelemetry.MatchPhase.TELEOP, vc.getVoltage());
                //GeneralTelemetry.logGamepad(logger, "Gamepad1", gamepad1);

                // Commit this cycle
                logger.flush();

                if(gamepad1.crossWasPressed())
                {
                    drive.setTargetHeading(0.0);
                }
                else if(gamepad1.triangleWasPressed())
                {
                    drive.setTargetHeading(Math.PI);
                }

                if(gamepad1.rightBumperWasPressed())
                {
                    drive.UpdatePIDCoefs();
                }

                double[] p = drive.getCurrentPose().toArray(); // [x, y, headingRad] in metres
                telemetry.addData("x (m)", "%.3f", p[0]);
                telemetry.addData("y (m)", "%.3f", p[1]);
                telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(p[2]));
                telemetry.update();
            }
        } finally {
            // Always cut power and close the log cleanly, even on stop/exception.
            drive.stop();
            drive.periodic();   // push the zero command out once
            logger.close();
        }
    }

    @NonNull
    private DriveIO getDriveIO() {
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
    private RobotLocalizer getLocalizer(RealtimeLogger logger) {
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