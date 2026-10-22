/*******************************************************************************
 * 
 * File        : SimWheelIO.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Hardware-agnostic AND drivetrain-agnostic abstraction 
 *               for the drive subsystem. The subsystem talks only to 
 *               this interface.
 * 
 * Authors     : AKA (06/2026), last update by AKA (06/2026) 
 *                                and inspired by Team 2910
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

package org.firstinspires.ftc.LyonLib.drive;

/**
 * Trivial wheel simulation: assumes the velocity controller is perfect and the
 * wheel instantly reaches the commanded speed. Good enough to test the
 * kinematics + subsystem on a laptop with no robot attached. Pure Java (no
 * WPILib), so it works in FRC and FTC unit tests alike.
 */
public class SimWheelIO implements WheelIO 
{
    private double commandedVelocity = 0.0;
    private double positionMeters = 0.0;
    private long lastNanos = System.nanoTime();

    @Override
    public void updateInputs(WheelIOInputs inputs) 
    {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1_000_000_000.0;
        lastNanos = now;

        positionMeters += commandedVelocity * dt;

        inputs.connected = true;
        inputs.velocityMetersPerSecond = commandedVelocity;
        inputs.positionMeters = positionMeters;
        inputs.appliedVolts = 0.0;
        inputs.statorCurrentAmps = 0.0;
        inputs.temperatureCelsius = 25.0;
    }

    @Override
    public void setVelocity(double metersPerSecond) 
    {
        commandedVelocity = metersPerSecond;
    }

    @Override
    public void setVoltage(double volts) 
    {
        commandedVelocity = 0.0;
    }
}
