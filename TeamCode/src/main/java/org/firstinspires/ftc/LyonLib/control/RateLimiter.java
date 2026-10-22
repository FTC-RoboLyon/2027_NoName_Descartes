/*******************************************************************************
 *  
 * File        : RateLimiter.java (v3.0)
 * Library     : LyonLib - FTC edition
 * Description : Rate Limiter class for controlling the rate of change of a value.
 *               Appropriate for applications where smooth transitions are required,
 *               such as voltage or velocity control.
 * 
 * Authors     : Gaspard (2023), last update by AKA (06/2026)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
package org.firstinspires.ftc.LyonLib.control;
 
import org.firstinspires.ftc.LyonLib.logging.DebugUtils;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
 
public class RateLimiter {
 
  private double dt = 0.02; // Default to 20ms in FRC, can be set by SetDeltaTime
  private double increasingRateLimit; // Rate limit for increasing speed (per cycle)
  private double brakingRateLimit; // Rate limit for deceleration when braking (per cycle)
  private double currentSpeed; // Current speed of the rate limiter
  private double targetSpeed; // Target speed to reach
 
  public RateLimiter() 
  {
    this.increasingRateLimit = 0.0;
    this.brakingRateLimit = 0.0;
    this.currentSpeed = 0.0;
    this.targetSpeed = 0.0;
  }
 
  public RateLimiter(double timeToReachMax) 
  {
    this.currentSpeed = 0.0;
    this.targetSpeed = 0.0;
    setRateLimit(timeToReachMax);
  }
 
  public RateLimiter(double timeToReachMaxUp, double timeToReachMaxDown) 
  {
    this.currentSpeed = 0.0;
    this.targetSpeed = 0.0;
    setRateLimit(timeToReachMaxUp, timeToReachMaxDown);
  }
 
  /**
   * @brief Sets the delta time for the RateLimiter and rescales the rate limits
   *        so that the physical ramp duration (timeToReachMax) is preserved.
   *
   * <p>Because the per-cycle rate limit is proportional to dt, changing dt simply
   * scales the existing limits by (newDt / oldDt). This keeps a zero limit at zero
   * (no division by the rate limit, so no Infinity/NaN when the limiter is
   * unconfigured).
   * 
   * @param deltaTime The new delta time value in seconds. Must be greater than 0.0.
   */
  public void setDeltaTime(double deltaTime) 
  {
    DebugUtils.assertion(deltaTime > 0.0, "Delta time must be positive");
 
    if (deltaTime <= 0.0)
      return; // Invalid dt: keep the previous configuration untouched.
 
    // m_dt is guaranteed > 0 here (default 0.02, only ever overwritten by a
    // positive value), so this scale is always finite and safe.
    double scale = deltaTime / dt;
 
    increasingRateLimit *= scale;
    brakingRateLimit *= scale;
    dt = deltaTime;
  }
 
  /**
   * @brief Sets the rate limit for the RateLimiter based on the time required to reach the maximum value.
   * 
   * This function calculates the rate limit for both upward and downward directions
   * based on the provided time to reach the maximum value.
   * 
   * @param timeToReachMax The time (in seconds) required to reach the maximum value. 
   *                       Must be greater than 0.0.
   * 
   * @warning If timeToReachMax is less than or equal to zero, rate limiting is disabled
   *          (the limiter passes the target straight through), and an error is logged.
   */
  public void setRateLimit(double timeToReachMax) 
  {
    DebugUtils.assertion(timeToReachMax > 0.0, "Time to reach max must be positive");
 
    if (timeToReachMax > 0.0) 
    {
      increasingRateLimit = this.dt / timeToReachMax; // Convert time to per-cycle rate limit
      brakingRateLimit = increasingRateLimit; // Same for both directions
    } 
    else 
    {
      DebugUtils.errorLog("RateLimiter: invalid timeToReachMax <= 0. Rate limiting disabled (pass-through).");
      increasingRateLimit = 0.0;
      brakingRateLimit = 0.0;
    }
  }
 
  /**
   * @brief Sets the rate limits for the RateLimiter.
   * 
   * This function configures the rate at which the value can increase or decrease
   * based on the specified time to reach the maximum value in either direction.
   * 
   * @param timeToReachMaxUp The time (in seconds) required to reach the maximum value when increasing.
   *                         Must be greater than 0.0.
   * @param timeToReachMaxDown The time (in seconds) required to reach the maximum value when stopping. 
   *                           Must be greater than 0.0.
   * 
   * @note If either parameter is less than or equal to 0.0, rate limiting is disabled
   *       (pass-through) and an error is logged.
   * 
   * @warning Ensure that both `timeToReachMaxUp` and `timeToReachMaxDown` are positive values to avoid invalid behavior.
   */
  public void setRateLimit(double timeToReachMaxUp, double timeToReachMaxDown) 
  {
    DebugUtils.assertion(timeToReachMaxUp > 0.0 && timeToReachMaxDown > 0.0,
        "Time to reach max must be positive");
 
    if (timeToReachMaxUp > 0.0 && timeToReachMaxDown > 0.0) 
    {
      increasingRateLimit = dt / timeToReachMaxUp; // Convert time to per-cycle rate limit
      brakingRateLimit = dt / timeToReachMaxDown; // Convert time to per-cycle rate limit
    } 
    else 
    {
      DebugUtils.assertion(false, "RateLimiter: invalid timeToReachMax <= 0. Rate limiting disabled (pass-through).");
      increasingRateLimit = 0.0;
      brakingRateLimit = 0.0;
    }
  }
 
  /**
   * Sets the target speed value for the rate limiter.
   *
   * @param target The new target speed value to be set.
   */
  public void setTarget(double target) 
  {
    this.targetSpeed = target;
  }
 
  /**
   * Sets the current speed value for the rate limiter.
   *
   * @param current The new current speed value to be set.
   */
  public void setCurrent(double current) 
  {
    this.currentSpeed = current;
  }
 
  public double getCurrentSpeed() 
  {
    return this.currentSpeed;
  }
 
  public double getTargetSpeed() 
  {
    return this.targetSpeed;
  }
 
  public double getRateLimitUp() 
  {
    return increasingRateLimit;
  }
 
  public double getRateLimitDown() 
  {
    return brakingRateLimit;
  }
 
  public double getDeltaTime() 
  {
    return dt;
  }
 
  /**
   * @brief Retrieves the current state of the Rate Limiter as a formatted string.
   * 
   * @return A string containing the formatted state of the Rate Limiter.
   */
  public String getState() 
  {
    StringBuilder state = new StringBuilder("RateLimiter State: \n");
 
    state.append("Current Speed: ").append(currentSpeed).append("\n");
    state.append("Target Speed: ").append(targetSpeed).append("\n");
    state.append("Rate Limit Up: ").append(increasingRateLimit).append("\n");
    state.append("Rate Limit Down: ").append(brakingRateLimit).append("\n");
    state.append("Delta Time: ").append(dt).append("\n");
    return state.toString();
  }
 
  /**
   * Updates the current speed towards the target speed, applying rate limits
   * for acceleration and deceleration.
   *
   * <p>Regime selection:
   * <ul>
   *   <li>Moving away from zero (step has the same sign as the current speed,
   *       or starting from rest) → accelerating → uses the increasing limit.</li>
   *   <li>Moving toward zero (step has the opposite sign to the current speed)
   *       → braking → uses the braking limit, and never crosses zero in a single
   *       step. The opposite-direction acceleration starts on the next cycle.</li>
   * </ul>
   *
   * <p>If the active limit is non-positive, rate limiting is disabled for that
   * regime and the target is reached immediately (pass-through).
   *
   * @return The updated current speed after applying rate limits.
   */
  public double update() 
  {
    double delta = targetSpeed - currentSpeed;
 
    // Already at target.
    if (delta == 0.0)
      return currentSpeed;
 
    // Accelerating when moving away from zero: from rest, or when the step has
    // the same sign as the current speed. Otherwise we are braking toward zero.
    boolean accelerating =
        (currentSpeed == 0.0) || (Math.signum(delta) == Math.signum(currentSpeed));
 
    double limit = accelerating ? increasingRateLimit : brakingRateLimit;
 
    // Non-positive limit → no limiting configured → pass straight through.
    if (limit <= 0.0) 
    {
      currentSpeed = targetSpeed;
      return currentSpeed;
    }
 
    // Bound the step by the active limit; clamp also prevents overshooting the
    // target when |delta| < limit.
    double step = UtilsRBL.clamp(-limit, delta, limit);
    double next = currentSpeed + step;
 
    // While braking, do not cross zero in a single step: stop exactly at zero.
    // The next cycle will accelerate in the opposite direction with the
    // increasing limit.
    if (!accelerating && next != 0.0 && (Math.signum(next) != Math.signum(currentSpeed))) 
      next = 0.0;
 
    currentSpeed = next;
    return currentSpeed;
  }
 
  /**
   * Updates the current speed towards the target speed, applying rate limits
   * for acceleration and deceleration.
   *
   * @param target The desired target speed.
   * @return The updated current speed after applying rate limits.
   */
  public double update(double target) 
  {
    targetSpeed = target; // Set the target speed
    return update();
  }
 
  /**
   * @brief Resets the RateLimiter's current and target speeds to zero.
   */
  public void reset() 
  {
    currentSpeed = 0.0;
    targetSpeed = 0.0;
  }

  /**
   * Resets the rate limiter with a new target speed and current speed.
   *
   * @param current The current speed to set.
   * @param target The desired target speed to set.
   */
  public void reset(double current, double target) 
  {
    targetSpeed = target;
    currentSpeed = current;
  }
}