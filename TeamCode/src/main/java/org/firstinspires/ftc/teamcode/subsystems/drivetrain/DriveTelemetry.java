package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.firstinspires.ftc.LyonLib.drive.DriveIO;
import org.firstinspires.ftc.LyonLib.drive.WheelIO;
import org.firstinspires.ftc.LyonLib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;

/**
 * Decides WHAT to log for the drive (the keys and which fields). Stays portable
 * by only touching plain data types and the {@link RealtimeLogger} seam — it
 * never knows whether the backend is NetworkTables or FtcDashboard.
 */
public final class DriveTelemetry 
{
    private DriveTelemetry() {}

    private static final String[] WHEEL_NAMES = {"FL", "FR", "RL", "RR"};

    public static void logCommanded(RealtimeLogger log, ChassisSpeeds commanded) 
    {
        log.log("Drivetrain/Commanded/vxMetersPerSec", commanded.vx);
        log.log("Drivetrain/Commanded/vyMetersPerSec", commanded.vy);
        log.log("Drivetrain/Commanded/omegaRadPerSec", commanded.omega);
    }

    public static void logInputs(RealtimeLogger log, DriveIO.DriveIOInputs inputs) 
    {
        // Pose as [x, y, headingRad] — point a Field2d / Odometry view at this key.
        log.log("Drivetrain/CurrentPose", inputs.currentPose.toArray());
        log.log("Drivetrain/HeadingDegrees", Math.toDegrees(inputs.currentPose.headingRadians));

//        log.log("Measured/vxMetersPerSec", inputs.measuredSpeeds.vx);
//        log.log("Measured/vyMetersPerSec", inputs.measuredSpeeds.vy);
//        log.log("Measured/omegaRadPerSec", inputs.measuredSpeeds.omega);
        //log.log("AllWheelsConnected", inputs.allWheelsConnected);

        for (int i = 0; i < inputs.wheels.length && i < WHEEL_NAMES.length; i++) 
        {
            WheelIO.WheelIOInputs w = inputs.wheels[i];
            String prefix = "Drivetrain/Wheels/" + WHEEL_NAMES[i] + "/";
            //log.log(prefix + "connected", w.connected);
            //log.log(prefix + "velocityMetersPerSec", w.velocityMetersPerSecond);
            //log.log(prefix + "positionMeters", w.positionMeters);
            log.log(prefix + "appliedVolts", w.appliedVolts);
            log.log(prefix + "statorCurrentAmps", w.statorCurrentAmps);
            //log.log(prefix + "temperatureCelsius", w.temperatureCelsius);
        }
    }
}
