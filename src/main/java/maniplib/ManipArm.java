package maniplib;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
import maniplib.utils.ManipArmConstants;
import maniplib.utils.ManipMath;
import maniplib.utils.PIDFConfig;

import static edu.wpi.first.units.Units.*;

public class ManipArm extends SubsystemBase {
    // Mutable holders for unit-safe values, persisted to avoid reallocation.
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutAngle angle = Rotations.mutable(0);
    private final MutAngularVelocity velocity = RPM.mutable(0);
    private final MutAngle absEncoderAngle = Rotations.mutable(0);
    // Mutable holder for a dimensionless value with no scaling. Should be clamped 0-1.
    // Triggers for when reaching max movements.
    private Trigger atMin;
    private Trigger atMax;
    private Trigger goingDown;
    private Trigger goingUp;
    // Booleans for limit switch functions.
    private boolean topLimitBoolean = false;
    private boolean bottomLimitBoolean = false;
    // Triggers for limit switch functions.
    private Trigger topLimit;
    private Trigger bottomLimit;
    // Various booleans to determine what to enable
    private boolean absSetup = false;
    private boolean isAdvancedEnabled = false;
    private boolean syncAbsEncoderInit = true;
    private boolean defaultCommandOverride = false;
    // Universal motor init
    private ManipMotor motor;
    private ArmFeedforward feedforward;
    private ManipArmConstants armConstants;
    // SysId Routine
    private SysIdRoutine sysIdRoutine;
    // Simulation class to help simulate what is going on, including gravity.
    private SingleJointedArmSim armSim;
    private Mechanism2d arm2d;
    // Mechanism for simulation, needs overridden for anything more than a basic arm.
    private MechanismLigament2d armMech;

    /**
     * Subsystem constructor, advanced {@link ManipArm} when config.kEnableAdvanced is set to true.
     */
    public ManipArm(ManipMotor motor, ManipArmConstants armConstants) {

        if (absSetup && syncAbsEncoderInit) {
            synchronizeAbsoluteEncoder();
        }

        if (!armConstants.kEnableAdvanced) {
            new ManipArm(motor);
        } else {
            this.motor = motor;
            this.armConstants = armConstants;
            this.isAdvancedEnabled = true;

            this.motor.configureMotor(
                    armConstants.kArmStallCurrentLimitAmps,
                    armConstants.kArmRampRate,
                    true,
                    armConstants.kArmInverted
            );

            this.topLimit = new Trigger(() -> topLimitBoolean);
            this.bottomLimit = new Trigger(() -> bottomLimitBoolean);

            this.atMin = new Trigger(() -> getAngle().isNear(this.armConstants.kMinAngle, Degrees.of(3)));
            this.atMax = new Trigger(() -> getAngle().isNear(this.armConstants.kMaxAngle, Degrees.of(3)));
            this.goingDown = new Trigger(() -> motor.getAppliedOutput() < 0);
            this.goingUp = new Trigger(() -> motor.getAppliedOutput() > 0);

            this.atMin.and(goingDown).or(topLimit).onTrue(run(this::stopArm));
            this.atMax.and(goingUp).or(topLimit).onTrue(run(this::stopArm));

            this.topLimit.onTrue(run(() ->
                    motor.setPosition((ManipMath.Arm.convertAngleToSensorUnits(
                            armConstants.kArmReduction,
                            armConstants.kMaxAngle)).in(Rotations))));
            this.bottomLimit.onTrue(run(() ->
                    motor.setPosition((ManipMath.Arm.convertAngleToSensorUnits(
                            armConstants.kArmReduction,
                            armConstants.kMinAngle)).in(Rotations))));

            this.motor.setGearbox(armConstants.gearbox);

            this.motor.setupRioPID(
                    new PIDFConfig(armConstants.kArmKp,
                            armConstants.kArmKi,
                            armConstants.kArmKd),
                    armConstants.kArmMaxVelocityRPM,
                    armConstants.kArmMaxAccelerationRPMperSecond,
                    0.01,
                    true
            );


            this.feedforward = new ArmFeedforward(
                    armConstants.kArmkS,
                    armConstants.kArmkG,
                    armConstants.kArmKv,
                    armConstants.kArmKa
            );

            this.sysIdRoutine = new SysIdRoutine(

                    new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(6), Seconds.of(30)),
                    new SysIdRoutine.Mechanism(
                            this::runArmVoltage,
                            log -> {
                                // Record a frame for the arm motor.
                                log.motor("manipArm")
                                        .voltage(appliedVoltage.mut_replace(motor.getAppliedOutput() *
                                                RobotController.getBatteryVoltage(), Volts))
                                        .angularPosition(angle.mut_replace(motor.getPosition(), Rotations))
                                        .angularVelocity(velocity.mut_replace(motor.getVelocity(), RPM));
                            },
                            this));

            this.armSim = new SingleJointedArmSim(
                    armConstants.gearbox,
                    armConstants.kArmReduction,
                    SingleJointedArmSim.estimateMOI(
                            armConstants.kArmLength,
                            armConstants.kArmMass),
                    armConstants.kArmLength,
                    armConstants.kMinAngle.in(Radians),
                    armConstants.kMaxAngle.in(Radians),
                    true,
                    armConstants.kArmStartingAngle.in(Radians),
                    0.02 / 4096.0,
                    0.0
            );

            // Mechanism 2d for simulation, needs overridden for anything more than a basic arm.
            arm2d = new Mechanism2d(
                    armConstants.kArmLength * 2,
                    armConstants.kArmLength * 2
            );

            // Mechanism root for simulation, needs overridden for anything more than a basic arm.
            MechanismRoot2d armRoot = arm2d.getRoot("ManipArm Root",
                    armConstants.kArmLength,
                    armConstants.kArmLength);

            this.armMech =
                    armRoot.append(
                            new MechanismLigament2d(
                                    "defaultManipArm",
                                    armConstants.kArmLength,
                                    armConstants.kArmStartingAngle.in(Degrees),
                                    6,
                                    new Color8Bit(Color.kOrange)
                            ));
        }
    }

    /**
     * Subsystem constructor, basic {@link ManipArm} with a {@link PIDFConfig}.
     */
    public ManipArm(ManipMotor motor, PIDFConfig pidfConfig) {
        this(motor);
        motor.configurePIDF(pidfConfig);
    }

    /**
     * Subsystem constructor, basic {@link ManipArm}.
     */
    public ManipArm(ManipMotor motor) {
        this.motor = motor;
    }

    @Override
    public void periodic() {
        if (Telemetry.manipVerbosity.ordinal() <= Telemetry.ManipTelemetry.LOW.ordinal()) {
            if (RobotBase.isSimulation()) {
                SmartDashboard.putData("Arm Side View", arm2d);
            }
        }
        if (Telemetry.manipVerbosity.ordinal() <= Telemetry.ManipTelemetry.HIGH.ordinal()) {
            SmartDashboard.putNumber("Arm Angle", getAngle().in(Degrees));
            SmartDashboard.putNumber("Arm Motor Rotations", motor.getPosition());
            SmartDashboard.putNumber("Arm Applied Output", motor.getAppliedOutput());

            SmartDashboard.putBoolean("Top Limit", topLimit.getAsBoolean());
            SmartDashboard.putBoolean("Bottom Limit", topLimit.getAsBoolean());
        }
    }

    /**
     * Ran periodically in simulation.
     * Controls the arm simulation.
     */
    @Override
    public void simulationPeriodic() {
        // Set the armSim input, we use volts for this.
        armSim.setInput(motor.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Update the arm sim, Standard loop time is 20ms.
        armSim.update(0.02);

        motor.iterateRevSim(
                RotationsPerSecond.of(ManipMath.Arm.convertAngleToSensorUnits(
                                armConstants.kArmReduction,
                                Radians.of(armSim.getVelocityRadPerSec())).in(Rotations))
                        .in(RPM),
                RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                0.02); // Time interval, in Seconds

        // Not implemented, will error when implemented.
        motor.iterateCTRESim();

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        motor.setPosition(ManipMath.Arm.convertAngleToSensorUnits(
                armConstants.kArmReduction,
                Radians.of(armSim.getAngleRads())).in(Rotations));

        motor.setVelocity(ManipMath.Arm.convertAngleToSensorUnits(
                armConstants.kArmReduction,
                Radians.of(armSim.getVelocityRadPerSec())).in(Rotations) * 60);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Arm Mechanism based on simulated arm angle
        armMech.setAngle(Degrees.convertFrom(armSim.getAngleRads(), Radians));
    }

    /**
     * @return The angle used to update an arm {@link MechanismLigament2d}.
     */
    public double getMechAngle() {
        return Degrees.convertFrom(armSim.getAngleRads(), Radians);
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
     * but recommended to keep arm position on boot. Can be called in init.
     * Value must be in rotations 0-1.
     */
    public void addAbsoluteEncoderValue(double absEncoderRotations) {
        absEncoderAngle.mut_replace(absEncoderRotations, Rotations);
        this.absSetup = true;
    }

    /**
     * Determines whether to sync the absolute encoder in the
     * {@link ManipArm} class or not. This is enabled by default.
     */
    public void setSyncAbsEncoderInit(boolean syncAbsEncoderInit) {
        if (absSetup) {
            this.syncAbsEncoderInit = syncAbsEncoderInit;
        } else {
            DriverStation.reportWarning("Absolute encoder for ManipArm is not set, cannot run setSyncAbsEncoderInit", true);
        }
    }

    /**
     * Seeds inbuilt encoder with absolute encoder value.
     * Syncs on init by default.
     */
    public void synchronizeAbsoluteEncoder() {
        motor.setPosition(
                Degrees.of(absEncoderAngle.in(Rotations))
                        .minus(armConstants.kArmOffsetToHorizantalZero)
                        .in(Rotations));
    }

    /**
     * Near the maximum Angle of the arm within X degrees.
     *
     * @param toleranceDegrees Degrees close to maximum of the Arm.
     * @return is near the maximum of the arm.
     */
    public boolean nearMax(double toleranceDegrees) {
        return getAngle().isNear(armConstants.kMaxAngle, Radians.convertFrom(toleranceDegrees, Degrees));

    }

    /**
     * Near the minimum angle of the Arm in within X degrees.
     *
     * @param toleranceDegrees Tolerance of the Arm.
     * @return is near the minimum of the arm.
     */
    public boolean nearMin(double toleranceDegrees) {
        return getAngle().isNear(armConstants.kMaxAngle, Radians.convertFrom(toleranceDegrees, Degrees));

    }

    /**
     * Runs the SysId routine to tune the Arm
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
            DriverStation.reportWarning("Advanced ManipArm is not setup, for safety SysID is disabled", true);
            return Commands.none();
        }
    }

    /**
     * Run the control loop to reach and maintain the setpoint from the preferences.
     */
    public void reachSetpoint(double setpoint) {
        if (isAdvancedEnabled) {
            double goalPosition = ManipMath.Arm.convertAngleToSensorUnits(armConstants.kArmReduction, Degrees.of(setpoint)).in(Rotations);
            double pidOutput = motor.getRioController().calculate(motor.getPosition(), goalPosition);
            TrapezoidProfile.State setpointState = motor.getRioController().getSetpoint();

            motor.setVoltage(pidOutput +
                    feedforward.calculate(setpointState.position,
                            setpointState.velocity));

        } else {
            motor.setReference(setpoint);
        }
    }

    /**
     * Basic method to run the arm at commanded speed.
     * This does not stop!!
     */
    public void runArmSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Basic method to run the arm at commanded voltage.
     * This does not stop!!
     */
    public void runArmVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }

    /**
     * Powers the motor with the kG feedforward value.
     * "Voltage required to counteract gravity".
     */
    public void runkG() {
        motor.setVoltage(armConstants.kArmkG);
    }

    /**
     * Powers the motor with the kG feedforward value as a command.
     * "Voltage required to counteract gravity".
     */
    public Command runkGCommand() {
        return run(this::runkG);
    }

    /**
     * Get the Angle of the Arm.
     *
     * @return Angle of the Arm.
     */
    public Angle getAngle() {
        if (isAdvancedEnabled) {
            angle.mut_replace(ManipMath.Arm.convertSensorUnitsToAngle(armConstants.kArmReduction, angle.mut_replace(motor.getPosition(), Rotations)));
        } else {
            DriverStation.reportWarning("Advanced ManipArm is required for getAngle()", true);
        }
        return angle;
    }

    /**
     * Get the velocity of Arm.
     *
     * @return Velocity of the Arm.
     */
    public AngularVelocity getVelocity() {
        if (!isAdvancedEnabled) {
            DriverStation.reportWarning("Advanced ManipArm is required for getVelocity(), returning 0", true);
        }
        return velocity.mut_replace(ManipMath.Arm.convertSensorUnitsToAngle(armConstants.kArmReduction, Rotations.of(motor.getVelocity())).per(Minute));
    }

    /**
     * Runs reachSetpoint as a {@link Command}.
     */
    public Command setGoal(double degrees) {
        return run(() -> reachSetpoint(degrees));
    }

    /**
     * Runs runArmSpeed as a {@link Command}.
     * This stops after command is finished.
     */
    public Command runArmSpeedCommand(double speed) {
        return runEnd(() -> runArmSpeed(speed), this::stopArm);
    }

    /**
     * Runs runArmVoltage as a {@link Command}.
     * This stops after command is finished.
     */
    public Command runArmVoltageCommand(Voltage volts) {
        return runEnd(() -> runArmVoltage(volts), this::stopArm);
    }

    /**
     * Sets the {@link Boolean} for when the top limit switch is hit for {@link ManipArm}.
     */
    public void setTopLimitSwitch(boolean topLimit) {
        this.topLimitBoolean = topLimit;
    }

    /**
     * Sets the {@link Boolean} for when the bottom limit switch is hit for {@link ManipArm}.
     */
    public void setBottomLimitSwitch(boolean bottomLimit) {
        this.bottomLimitBoolean = bottomLimit;
    }

    /**
     * A command to be used as a default command to stow the arm.
     * Use toggleAutoStow() to toggle it on and off.
     * It's good for if you want it to stow but want safety or to be able to control manually.
     */
    public Command autoStowWithOverride(double stowAngle) {
        return run(() -> {
            if (!defaultCommandOverride) {
                reachSetpoint(stowAngle);
            } else {
                Commands.none();
            }
        });
    }

    /**
     * Toggles auto-stow of defaultCommandOverride
     */
    public void toggleAutoStow() {
        this.defaultCommandOverride = !defaultCommandOverride;
    }

    public void setAutoStow(boolean autoStow) {
        this.defaultCommandOverride = autoStow;
    }

    /**
     * Stops the arm.
     */
    public void stopArm() {
        runArmSpeed(0.0);
    }
}
