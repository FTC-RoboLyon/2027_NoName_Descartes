/*******************************************************************************
 * 
 * File        : DriveIO.java (v1.1)
 * Library     : LyonLib - FTC edition
 * Description : Hardware-agnostic AND drivetrain-agnostic abstraction 
 *               for the drive subsystem. The subsystem talks only to 
 *               this interface.
 * 
 * Authors     : AKA (06/2026), last update by VTT (07/2026)
 *                                and inspired by Team 2910
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

package org.firstinspires.ftc.LyonLib.drive;

import org.firstinspires.ftc.LyonLib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.LyonLib.kinematics.DrivetrainKinematics;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;

/**
 * Hardware-agnostic AND drivetrain-agnostic abstraction. The subsystem talks
 * only to this interface, in {@link ChassisSpeeds}.
 * The implementation can be a real mecanum drive, a swerve, or even a tank drive.
 */
public interface DriveIO 
{

    /** Telemetry holder, populated each loop by the implementation. */
    class DriveIOInputs 
    {
        public ChassisSpeeds measuredSpeeds = new ChassisSpeeds();
        public Pose2d currentPose = new Pose2d();
        public boolean allWheelsConnected = false;
        public WheelIO.WheelIOInputs[] wheels = new WheelIO.WheelIOInputs[0];
    }

    void updateInputs(DriveIOInputs inputs);

    /** Applies robot-relative chassis speeds to the hardware. */
    void applyChassisSpeeds(ChassisSpeeds robotRelativeSpeeds);

    /** Resets the estimated field pose (start of a match / simulation). */
    default void resetPose(Pose2d pose) {}

    /** Stops all motion. */
    default void stop() 
    {
        applyChassisSpeeds(new ChassisSpeeds());
    }

    /** Returns the Drivetrain kinematics */
    DrivetrainKinematics getDrivetrainKinematics();

    /** Returns the wheel inputs tab object (not a copy) */
    WheelIO.WheelIOInputs[] getWheelInputs();
}
