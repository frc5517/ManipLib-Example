package maniplib.motors;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import maniplib.utils.PIDControlType;
import maniplib.utils.PIDFConfig;

/**
 * Swerve motor abstraction which defines a standard interface for motors within a swerve module.
 */
public abstract class ManipMotor {

    /*
    Custom methods needed for each controller :
    getSimMotor, custom simMotor code for each added motor support.
     */

    /**
     * The maximum amount of times the swerve motor will attempt to configure a motor if failures occur.
     */
    public final int maximumRetries = 5;

    /**
     * Configure the factory defaults.
     */
    public abstract void factoryDefaults();

    /**
     * Clear the sticky faults on the motor controller.
     */
    public abstract void clearStickyFaults();

    /**
     * Used to pass the mechanism gearbox to the motor for sim.
     */
    public abstract void setGearbox(DCMotor gearbox);

    /**
     * @return The applied output for sim use.
     */
    public abstract double getSimAppliedOutput();

    /**
     * Iterates Rev's sim, does nothing on CTRE devices.
     */
    public abstract void iterateRevSim(double velocity, double vbus, double dt);

    /**
     * Not implemented, does nothing on Rev devices,
     */
    public abstract void iterateCTRESim();

    /**
     * Sets up the {@link ManipSparkMax} to use rioPID.
     *
     * @param pidfConfig      pid settings to use.
     * @param maxVelocity     maximum velocity for trapezoid profiling.
     * @param maxAcceleration maximum acceleration for trapezoid profiling.
     * @param useRioPID       boolean to enable rioPID.
     */
    public abstract void setupRioPID(PIDFConfig pidfConfig, double maxVelocity, double maxAcceleration, double tolerance, boolean useRioPID);

    /**
     * Whether to use rioPID or revPID
     *
     * @param useRioPID boolean to enable rioPID
     */
    public abstract void useRioPID(boolean useRioPID);

    public abstract ProfiledPIDController getRioController();

    /**
     * Sets the {@link PIDControlType} to use on the motor.
     */
    public abstract void setPIDControlType(PIDControlType.ControlType controlType);

    /**
     * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
     *
     * @param config Configuration class holding the PIDF values.
     */
    public abstract void configurePIDF(PIDFConfig config);

    /**
     * Configure the PID wrapping for the position closed loop controller.
     *
     * @param minInput Minimum PID input.
     * @param maxInput Maximum PID input.
     */
    public abstract void configurePIDWrapping(double minInput, double maxInput);

    /**
     * Set the idle mode.
     *
     * @param isBrakeMode Set the brake mode.
     */
    public abstract void setMotorBrake(boolean isBrakeMode);

    /**
     * Set the motor to be inverted.
     *
     * @param inverted State of inversion.
     */
    public abstract void setInverted(boolean inverted);

    /**
     * Save the configurations from flash to EEPROM.
     */
    public abstract void burnFlash();

    /**
     * Sets the {@link ManipMotor} to follow another {@link ManipMotor}.
     *
     * @param leadMotor  lead {@link ManipMotor} to follow.
     * @param isInverted whether to invert the follower or not.
     */
    public abstract void setAsFollower(ManipMotor leadMotor, Boolean isInverted);

    /**
     * Set the percentage output.
     *
     * @param percentOutput percent out for the motor controller.
     */
    public abstract void set(double percentOutput);

    /**
     * Set the closed loop PID controller reference point.
     *
     * @param setpoint    Setpoint in meters per second or angle in degrees.
     * @param feedforward Feedforward in volt-meter-per-second or kV.
     */
    public abstract void setReference(double setpoint, double feedforward);

    /**
     * Set the closed loop PID controller reference point.
     *
     * @param setpoint Setpoint in meters per second or angle in degrees.
     */
    public abstract void setReference(double setpoint);

    /**
     * Stop the motor.
     */
    public abstract void stopMotor();

    /**
     * A command to stop the motor.
     *
     * @return a command to stop the motor.
     */
    public abstract Command stopMotorCommand();

    /**
     * Get the voltage output of the motor controller.
     *
     * @return Voltage output.
     */
    public abstract double getVoltage();

    /**
     * Set the voltage of the motor.
     *
     * @param voltage Voltage to set.
     */
    public abstract void setVoltage(double voltage);

    /**
     * Set the voltage of the motor using {@link Voltage} units.
     *
     * @param voltage units to set the motor with.
     */
    public abstract void setVoltage(Voltage voltage);

    /**
     * Returns the canid of the motor.
     *
     * @return the canid of the motor.
     */
    public abstract int getMotorID();

    /**
     * Get the applied dutycycle output.
     *
     * @return Applied dutycycle output to the motor.
     */
    public abstract double getAppliedOutput();

    /**
     * Get the velocity of the integrated encoder.
     *
     * @return velocity in meters per second or degrees per second.
     */
    public abstract double getVelocity();

    /**
     * Get the position of the integrated encoder.
     *
     * @return Position in meters or degrees.
     */
    public abstract double getPosition();

    /**
     * Set the integrated encoder velocity.
     *
     * @param velocity Integrated encoder velocity. Should be meters per second or degrees per second.
     */
    public abstract void setVelocity(double velocity);

    /**
     * Set the integrated encoder position.
     *
     * @param position Integrated encoder position. Should be angle in degrees or meters per second.
     */
    public abstract void setPosition(double position);

    /**
     * Set the voltage compensation for the swerve module motor.
     *
     * @param nominalVoltage Nominal voltage for operation to output to.
     */
    public abstract void setVoltageCompensation(double nominalVoltage);

    /**
     * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
     * voltage compensation. This is useful to protect the motor from current spikes.
     *
     * @param currentLimit Current limit in AMPS at free speed.
     */
    public abstract void setCurrentLimit(int currentLimit);

    /**
     * Set the maximum rate the open/closed loop output can change by.
     *
     * @param rampRate Time in seconds to go from 0 to full throttle.
     */
    public abstract void setLoopRampRate(double rampRate);

    /**
     * Get the motor object from the module.
     *
     * @return Motor object.
     */
    public abstract Object getMotor();
}
