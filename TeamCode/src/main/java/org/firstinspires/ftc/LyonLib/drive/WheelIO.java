/*******************************************************************************
 * 
 * File        : WheelIO.java (v1.0)
 * Library     : LyonLib - FTC edition
 * Description : Hardware abstraction for a single driven wheel.
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
 * Hardware abstraction for a single driven wheel. This is the FRC &lt;-&gt; FTC swap
 * boundary: the only thing you rewrite to move platforms is an implementation
 * of this interface (e.g. {@code SparkWheelIO} for FRC, a {@code DcMotorEx}-based
 * one for FTC). Everything else in the drive package stays the same.
 *
 * <p>The contract is expressed entirely in SI units (meters, m/s, volts) so the
 * kinematics and subsystem layers stay unit-portable. Each implementation
 * converts to/from its own motor units internally.
 */
public interface WheelIO 
{

    class WheelIOInputs 
    {
        public boolean connected = false;
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    /** Reads the latest sensor values into the given inputs object. */
    void updateInputs(WheelIOInputs inputs);

    /** Commands a wheel velocity in meters/second (open- or closed-loop). */
    void setVelocity(double metersPerSecond);

    /** Open-loop voltage, mainly for characterization. */
    default void setVoltage(double volts) {}

    /** Stops the wheel. */
    default void stop() 
    {
        setVoltage(0.0);
    }
}
