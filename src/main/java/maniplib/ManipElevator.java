package maniplib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import maniplib.motors.ManipMotor;
import maniplib.utils.ManipElevatorConstants;
import maniplib.utils.ManipMath;
import maniplib.utils.PIDFConfig;

import static edu.wpi.first.units.Units.*;

public class ManipElevator extends SubsystemBase {
    // Mutable holders for unit-safe values, persisted to avoid reallocation.
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);
    private final MutDistance distance = Meters.mutable(0);
    private final MutAngle absEncoderAngle = Rotations.mutable(0);
    // Triggers for when reaching max movements.
    private Trigger atMin;
    private Trigger atMax;
    // Triggers for limit switch functions.
    private Trigger topLimitHit;
    private Trigger bottomLimitHit;
    // Various booleans to determine what to enable
    private boolean absSetup = false;
    private boolean isAdvancedEnabled = false;
    private boolean syncAbsEncoderInit = true;
    private boolean defaultCommandOverride = false;
    // Universal motor init
    private final ManipMotor motor;
    private ElevatorFeedforward feedforward;
    private ManipElevatorConstants elevatorConstants;
    // SysId Routine
    private SysIdRoutine sysIdRoutine;
    // Simulation class to help simulate what is going on, including gravity.
    private ElevatorSim elevatorSim;
    private Mechanism2d elevator2d;
    // Mechanism for simulation, needs overridden for anything more than a basic elevator.
    private MechanismLigament2d elevatorMech;

    /**
     * Subsystem constructor, advanced {@link ManipElevator} when config.kEnableAdvanced is set to true.
     */
    public ManipElevator(ManipMotor motor, ManipElevatorConstants config) {

        if (absSetup && syncAbsEncoderInit) {
            synchronizeAbsoluteEncoder();
        }

        if (!config.kEnableAdvanced) {
            this.motor = motor;
        } else {
            this.motor = motor;
            this.elevatorConstants = config;
            this.isAdvancedEnabled = true;

            this.atMin = new Trigger(() -> getLinearPosition().isNear(config.kMinHeight, Inches.of(1)));
            this.atMax = new Trigger(() -> getLinearPosition().isNear(config.kMaxHeight, Inches.of(1)));

            this.motor.setGearbox(elevatorConstants.gearbox);

            this.motor.configureMotor(
                    elevatorConstants.kElevatorCurrentLimit,
                    elevatorConstants.kElevatorRampRate,
                    true,
                    elevatorConstants.kIsInverted
                );

            motor.setupRioPID(
                    new PIDFConfig(config.kElevatorKp,
                            config.kElevatorKi,
                            config.kElevatorKd),
                    config.kMaxVelocity,
                    config.kMaxAcceleration,
                    0.01,
                    true
            );

            this.feedforward = new ElevatorFeedforward(
                    config.kElevatorkS,
                    config.kElevatorkG,
                    config.kElevatorkV,
                    config.kElevatorkA
            );

            this.sysIdRoutine = new SysIdRoutine(

                    new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(6), Seconds.of(30)),
                    new SysIdRoutine.Mechanism(
                            this::runElevatorVoltage,
                            log -> {
                                // Record a frame for the elevator motor.
                                log.motor("manipElevator")
                                        .voltage(appliedVoltage.mut_replace(motor.getAppliedOutput() *
                                                RobotController.getBatteryVoltage(), Volts))
                                        .linearPosition(distance.mut_replace(getHeightMeters(),
                                                Meters)) // Records Height in Meters via SysIdRoutineLog.linearPosition
                                        .linearVelocity(velocity.mut_replace(getVelocityMetersPerSecond(),
                                                MetersPerSecond)); // Records velocity in MetersPerSecond via SysIdRoutineLog.linearVelocity
                            },
                            this));

            this.elevatorSim = new ElevatorSim(
                    config.gearbox,
                    config.kElevatorGearing,
                    config.kElevatorCarriageMass,
                    config.kElevatorDrumRadius,
                    config.kMinHeight.in(Meters),
                    config.kMaxHeight.in(Meters),
                    true,
                    config.kStartingHeightSim.in(Meters),
                    0.01,
                    0.0
            );

            // Mechanism 2d for simulation, needs overridden for anything more than a basic elevator.
            this.elevator2d = new Mechanism2d(
                    (config.kMaxHeight.in(Meters) / 5),
                    config.kMaxHeight.in(Meters) + (config.kMaxHeight.in(Meters) / 10)
            );

            // Mechanism root for simulation, needs overridden for anything more than a basic elevator.
            MechanismRoot2d elevatorRoot = elevator2d.getRoot("ManipElevator Root",
                    (config.kMaxHeight.in(Meters) / 10),
                    config.kMinHeight.in(Meters));

            this.elevatorMech =
                    elevatorRoot.append(
                            new MechanismLigament2d(
                                    "defaultManipElevator",
                                    config.kStartingHeightSim.in(Meters),
                                    90,
                                    6,
                                    new Color8Bit(Color.kYellow)
                            ));
        }

    }

    @Override
    public void periodic() {
        if (Telemetry.manipVerbosity.ordinal() <= Telemetry.ManipTelemetry.LOW.ordinal()) {
            SmartDashboard.putData("Elevator Side", elevator2d);
        }
        if (Telemetry.manipVerbosity.ordinal() <= Telemetry.ManipTelemetry.HIGH.ordinal()) {
            SmartDashboard.putNumber("Elevator Height", getLinearPosition().in(Inches));
            SmartDashboard.putNumber("Elevator Applied Output", motor.getAppliedOutput());
        }
    }

    /**
     * Subsystem constructor, basic {@link ManipElevator} with a {@link PIDFConfig}.
     */
    public ManipElevator(ManipMotor motor, PIDFConfig pidfConfig) {
        this(motor);
        motor.configurePIDF(pidfConfig);
    }

    /**
     * Subsystem constructor, basic {@link ManipElevator}.
     */
    public ManipElevator(ManipMotor motor) {
        this.motor = motor;
    }

    /**
     * Ran periodically in simulation.
     * Controls the elevator simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Set the elevatorSim input, we use volts for this.
        elevatorSim.setInput(motor.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Update the elevator sim, Standard loop time is 20ms.
        elevatorSim.update(0.02);

        motor.iterateRevSim(
                ManipMath.Elevator.convertDistanceToRotations(
                        elevatorConstants.kElevatorDrumRadius,
                        elevatorConstants.kElevatorGearing,
                        Meters.of(elevatorSim.getVelocityMetersPerSecond())).per(Second).in(RPM),
                RoboRioSim.getVInVoltage(),
                0.020);

        // Not implemented will error
        motor.iterateCTRESim();

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        motor.setPosition(ManipMath.Elevator.convertDistanceToRotations(
                elevatorConstants.kElevatorDrumRadius,
                elevatorConstants.kElevatorGearing,
                Meters.of(elevatorSim.getPositionMeters())).in(Rotations));

        motor.setVelocity(ManipMath.Elevator.convertDistanceToRotations(
                elevatorConstants.kElevatorDrumRadius,
                elevatorConstants.kElevatorGearing,
                Meters.of(elevatorSim.getVelocityMetersPerSecond())).in(Rotations) * 60);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

        // Update the Elevator Mechanism based on simulated elevator height
        elevatorMech.setLength(getLinearPosition().in(Meters));
    }

    /**
     * @return The length used to update a elevator {@link MechanismLigament2d}
     */
    public double getMechLength() {
        return getLinearPosition().in(Meters);
    }

    /**
     * Sets the {@link ManipMotor} to follow another {@link ManipMotor}.
     *
     * @param followerMotor {@link ManipMotor} to follow the lead motor.
     * @param isInverted    whether to invert the follower or not.
     */
    public void addFollower(ManipMotor followerMotor, boolean isInverted) {
        followerMotor.setAsFollower(motor, isInverted);
    }

    /**
     * Adds an absolute encoder to sync to on init. This is not used for actual control
     * but recommended to keep elevator position on boot. Can be called in init.
     */
    public void addAbsoluteEncoderValue(double absEncoderDegrees) {
        absEncoderAngle.mut_replace(absEncoderDegrees - elevatorConstants.kAbsEncoderOffset, Degrees);
        this.absSetup = true;
    }

    /**
     * Determines whether to sync the absolute encoder in the
     * {@link ManipElevator} class or not. This is enabled by default.
     */
    public void setSyncAbsEncoderInit(boolean syncAbsEncoderInit) {
        if (absSetup) {
            this.syncAbsEncoderInit = syncAbsEncoderInit;
        } else {
            DriverStation.reportWarning("Absolute encoder for ManipElevator is not set, cannot run setSyncAbsEncoderInit", true);
        }
    }

    /**
     * Seeds inbuilt encoder with absolute encoder value.
     * Syncs on init by default.
     */
    public void synchronizeAbsoluteEncoder() {
        motor.setPosition(Rotations.of(absEncoderAngle.in(Degrees)).in(Rotations));
    }

    /**
     * Runs the SysId routine to tune the elevator
     *
     * @return SysId Routine command
     */
    public Command runSysIdRoutine() {
        if (isAdvancedEnabled) {
            return (sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atMax))
                    .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atMin))
                    .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atMax))
                    .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atMin))
                    .andThen(Commands.print("DONE"));
        } else {
            DriverStation.reportWarning("Advanced ManipElevator is not setup, for safety SysID is disabled", true);
            return Commands.none();
        }
    }

    /**
     * Get Elevator Velocity
     *
     * @return Elevator Velocity
     */
    public LinearVelocity getLinearVelocity()
    {
        return ManipMath.Elevator.convertRotationsToDistance(
                elevatorConstants.kElevatorDrumRadius,
                elevatorConstants.kElevatorGearing,
                Rotations.of(motor.getVelocity())).per(Minute);
    }

    /**
     * Get the height of the Elevator
     *
     * @return Height of the elevator
     */
    public Distance getLinearPosition()
    {
        return ManipMath.Elevator.convertRotationsToDistance(
                elevatorConstants.kElevatorDrumRadius,
                elevatorConstants.kElevatorGearing,
                Rotations.of(motor.getPosition()));
    }

    /**
     * Get the height in meters.
     *
     * @return Height in meters
     */
    public double getHeightMeters()
    {
        return (motor.getPosition() / elevatorConstants.kElevatorGearing) *
                (2 * Math.PI * elevatorConstants.kElevatorDrumRadius);
    }

    /**
     * The velocity of the elevator in meters per second.
     *
     * @return velocity in meters per second
     */
    public double getVelocityMetersPerSecond()
    {
        return ((motor.getVelocity() / 60)/ elevatorConstants.kElevatorGearing) *
                (2 * Math.PI * elevatorConstants.kElevatorDrumRadius);
    }

    /**
     * A trigger for when the height is at an acceptable tolerance.
     *
     * @param height    Height in Meters
     * @param tolerance Tolerance in meters.
     * @return {@link Trigger}
     */
    public Trigger atHeight(double height, double tolerance)
    {
        return new Trigger(() -> MathUtil.isNear(height,
                getHeightMeters(),
                tolerance));
    }

    /**
     * Near the maximum height of the elevator within X millimeters.
     *
     * @param toleranceMillimeters Tolerance of the Elevator.
     * @return is near the maximum of the elevator.
     */
    public boolean nearMax(double toleranceMillimeters) {
        return getLinearPosition().isNear(elevatorConstants.kMaxHeight, Meters.convertFrom(toleranceMillimeters, Millimeters));

    }

    /**
     * Near the minimum height of the elevator in within X millimeters.
     *
     * @param toleranceMillimeters Tolerance of the Elevator.
     * @return is near the minimum of the elevator.
     */
    public boolean nearMin(double toleranceMillimeters) {
        return getLinearPosition().isNear(elevatorConstants.kMinHeight, Meters.convertFrom(toleranceMillimeters, Millimeters));

    }

    /**
     * Run the control loop to reach and maintain the setpoint from the preferences.
     * If using basic control setpoint is in sensor units.
     */
    public void reachSetpoint(double setpointInches) {
        if (isAdvancedEnabled) {
            limitSwitchFunction();

            motor.setVoltage(MathUtil.clamp(
                    motor.getRioController().calculate(getHeightMeters(), Meters.convertFrom(setpointInches, Inches)) +
                            feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                    motor.getRioController().getSetpoint().velocity), -7, 7));

        } else {
            limitSwitchFunction();
            motor.setReference(setpointInches);
        }
    }

    /**
     * Basic method to run the elevator at commanded speed.
     */
    public void runElevatorSpeed(double speed) {
        limitSwitchFunction();
        motor.set(-speed);
    }

    /**
     * Basic method to run the elevator at commanded voltage.
     */
    public void runElevatorVoltage(Voltage volts) {
        limitSwitchFunction();
        motor.setVoltage(volts);
    }

    /**
     * Runs reachSetpoint as a {@link Command}.
     */
    public Command setGoal(double setpointInches) {
        return run(() -> reachSetpoint(setpointInches));
    }

    /**
     * Runs runElevatorSpeed as a {@link Command}.
     */
    public Command runElevatorSpeedCommand(double speed) {
        return run(() -> runElevatorSpeed(speed));
    }

    /**
     * Runs runElevatorVoltage as a {@link Command}.
     */
    public Command runElevatorVoltageCommand(Voltage volts) {
        return run(() -> runElevatorVoltage(volts));
    }

    /**
     * Sets the {@link Trigger} for when the top limit switch is hit for {@link ManipElevator}.
     *
     * @param topLimitHit top limit switch {@link Trigger}.
     */
    public void setTopLimitSwitch(Trigger topLimitHit) {
        this.topLimitHit = topLimitHit;
    }

    /**
     * Sets the {@link Trigger} for when the bottom limit switch is hit for {@link ManipElevator}.
     *
     * @param bottomLimitHit bottom limit switch {@link Trigger}.
     */
    public void setBottomLimitSwitch(Trigger bottomLimitHit) {
        this.bottomLimitHit = bottomLimitHit;
    }

    /**
     * Function that sees if there's active limit switches then stops the {@link ManipElevator} if one is hit.
     * Also sets soft limits based off of given Min and Max positions.
     */
    public void limitSwitchFunction() {

        if (atMax.getAsBoolean() || atMin.getAsBoolean()) {
            stopElevator();
        }

        if (topLimitHit != null) {
            if (motor.getAppliedOutput() > 0 && bottomLimitHit.getAsBoolean()) {
                stopElevator();
            } else {
                Commands.none(); // Stop stopping the arm
            }
        }
        if (bottomLimitHit != null) {
            if (motor.getAppliedOutput() < 0 && bottomLimitHit.getAsBoolean()) {
                stopElevator();
            } else {
                Commands.none(); // Stop stopping the arm
            }
        }
    }

    /**
     * A command to be used as a default command to stow the elevator.
     * Use toggleAutoStow() to toggle it on and off.
     * It's good for if you want it to stow but want safety or to be able to control manually.
     */
    public Command autoStowWithOverride(double stowHeight) {
        return run(() -> {
            if (!defaultCommandOverride) {
                reachSetpoint(stowHeight);
            } else {
                Commands.none();
            }
        });
    }

    /**
     * Toggles auto-stow of defaultCommandOverride
     */
    public Command toggleAutoStow() {
        return run(() -> {
            defaultCommandOverride = !defaultCommandOverride;
        });
    }

    /**
     * Stops the elevator.
     */
    public void stopElevator() {
        motor.stopMotor();
    }
}
