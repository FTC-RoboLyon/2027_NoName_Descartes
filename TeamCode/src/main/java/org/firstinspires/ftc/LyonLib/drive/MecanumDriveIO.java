/*******************************************************************************
 * 
 * File        : MecanumDriveIO.java (v1.1)
 * Library     : LyonLib - FTC edition
 * Description : Mecanum implementation of {@link DriveIO}. It imports 
 *              four {@link WheelIO} objects, and the kinematics.
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
import org.firstinspires.ftc.LyonLib.kinematics.MecanumKinematics;
import org.firstinspires.ftc.LyonLib.kinematics.MecanumWheelSpeeds;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConstants;

/**
 * Mecanum implementation of {@link DriveIO}.
 * imports. It owns the kinematics, four {@link WheelIO} objects, and the wheel
 * odometry that estimates the field pose.
 */
public class MecanumDriveIO implements DriveIO 
{
    private final WheelIO frontLeft;
    private final WheelIO frontRight;
    private final WheelIO rearLeft;
    private final WheelIO rearRight;

    private final MecanumKinematics kinematics;
    private final double maxWheelSpeed; //in meter per second

    private final WheelIO.WheelIOInputs[] wheelInputs = new WheelIO.WheelIOInputs[] 
    {
        new WheelIO.WheelIOInputs(),
        new WheelIO.WheelIOInputs(),
        new WheelIO.WheelIOInputs(),
        new WheelIO.WheelIOInputs()
    };

    // ---- Odometry state ----
    private Pose2d pose = new Pose2d();
    private final double[] lastWheelPositions = new double[4];
    private boolean poseInitialized = false;

    public MecanumDriveIO(
            WheelIO frontLeft,
            WheelIO frontRight,
            WheelIO rearLeft,
            WheelIO rearRight,
            MecanumKinematics kinematics,
            double maxWheelSpeedMetersPerSecond) 
    {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.kinematics = kinematics;
        this.maxWheelSpeed = maxWheelSpeedMetersPerSecond;
    }

    @Override
    public void applyChassisSpeeds(ChassisSpeeds robotSpeeds) 
    {
        MecanumWheelSpeeds wheels = kinematics.toWheelSpeeds(robotSpeeds);
        wheels.desaturate(maxWheelSpeed);

        frontLeft.setVelocity(wheels.frontLeft* DrivetrainConstants.FRONT_LEFT_COEF);
        frontRight.setVelocity(wheels.frontRight*DrivetrainConstants.FRONT_RIGHT_COEF);
        rearLeft.setVelocity(wheels.rearLeft*DrivetrainConstants.BACK_LEFT_COEF);
        rearRight.setVelocity(wheels.rearRight*DrivetrainConstants.BACK_RIGHT_COEF);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) 
    {
        frontLeft.updateInputs(wheelInputs[0]);
        frontRight.updateInputs(wheelInputs[1]);
        rearLeft.updateInputs(wheelInputs[2]);
        rearRight.updateInputs(wheelInputs[3]);

        double[] positions = 
        {
            wheelInputs[0].positionMeters,
            wheelInputs[1].positionMeters,
            wheelInputs[2].positionMeters,
            wheelInputs[3].positionMeters
        };

        // First cycle (or just after a reset): baseline the positions so the
        // first delta is zero and the pose doesn't jump.
        if (!poseInitialized) {
            System.arraycopy(positions, 0, lastWheelPositions, 0, 4);
            poseInitialized = true;
        }
        //TODO : implement odometry and localisation

        // Wheel position deltas -> robot-frame twist via forward kinematics
        // (the same linear map applies to displacements as to velocities).
        MecanumWheelSpeeds delta = new MecanumWheelSpeeds(
                positions[0] - lastWheelPositions[0],
                positions[1] - lastWheelPositions[1],
                positions[2] - lastWheelPositions[2],
                positions[3] - lastWheelPositions[3]);
        ChassisSpeeds twist = kinematics.toChassisSpeeds(delta);
        pose = pose.exp(
                twist.vx, twist.vy, twist.omega);

        System.arraycopy(positions, 0, lastWheelPositions, 0, 4);
        inputs.currentPose = pose;

        // Measured chassis speeds (instantaneous) from wheel velocities.
        MecanumWheelSpeeds measured = new MecanumWheelSpeeds(
                wheelInputs[0].velocityMetersPerSecond,
                wheelInputs[1].velocityMetersPerSecond,
                wheelInputs[2].velocityMetersPerSecond,
                wheelInputs[3].velocityMetersPerSecond);
        inputs.measuredSpeeds = kinematics.toChassisSpeeds(measured);

        inputs.wheels = wheelInputs;
        inputs.allWheelsConnected =
                wheelInputs[0].connected
                        && wheelInputs[1].connected
                        && wheelInputs[2].connected
                        && wheelInputs[3].connected;
    }

    @Override
    public void resetPose(Pose2d newPose) 
    {
        this.pose = newPose;
        // Re-baseline wheel positions on the next update so there is no jump.
        this.poseInitialized = false;
    }

    @Override
    public DrivetrainKinematics getDrivetrainKinematics()
    {
        return kinematics;
    }

    @Override
    public WheelIO.WheelIOInputs[] getWheelInputs()
    {
        return wheelInputs;
    }
}
