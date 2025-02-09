package maniplib;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
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
    private ManipMotor motor;
    private ArmFeedforward feedforward;
    private ManipArmConstants armConstants;
    // SysId Routine
    private SysIdRoutine sysIdRoutine;
    // Simulation class to help simulate what is going on, including gravity.
    private SingleJointedArmSim armSim;
    // Mechanism 2d for simulation, needs overridden for anything more than a basic arm.
    private Mechanism2d arm2d;
    // Mechanism root for simulation, needs overridden for anything more than a basic arm.
    private MechanismRoot2d armRoot;
    // Mechanism for simulation, needs overridden for anything more than a basic arm.
    private MechanismLigament2d armMech;

    /**
     * Subsystem constructor, advanced {@link ManipArm} when config.kEnableAdvanced is set to true.
     */
    public ManipArm(ManipMotor motor, ManipArmConstants config) {

        if (absSetup && syncAbsEncoderInit) {
            synchronizeAbsoluteEncoder();
        }

        if (!config.kEnableAdvanced) {
            new ManipArm(motor);
        } else {
            this.motor = motor;
            this.armConstants = config;
            this.isAdvancedEnabled = true;

            this.atMin = new Trigger(() -> getAngle().isNear(armConstants.kMinAngle, Degrees.of(3)));
            this.atMax = new Trigger(() -> getAngle().isNear(armConstants.kMaxAngle, Degrees.of(3)));

            motor.setupRioPID(
                    new PIDFConfig(config.kArmKp,
                            config.kArmKi,
                            config.kArmKd),
                    config.kArmMaxVelocityRPM,
                    config.kArmMaxAccelerationRPMperSecond,
                    0.01,
                    true
            );

            this.feedforward = new ArmFeedforward(
                    config.kArmkS,
                    config.kArmkG,
                    config.kArmKv,
                    config.kArmKa
            );

            this.sysIdRoutine = new SysIdRoutine(
                    new SysIdRoutine.Config(Volts.per(Second).of(config.kArmRampRate), Volts.of(6), Seconds.of(5)),
                    new SysIdRoutine.Mechanism(
                            motor::setVoltage,
                            log -> {
                                log.motor("manipArm")
                                        .voltage(
                                                appliedVoltage.mut_replace(motor.getAppliedOutput() *
                                                        RobotController.getBatteryVoltage(), Volts))
                                        .angularPosition(angle.mut_replace(motor.getPosition(), Rotations))
                                        .angularVelocity(velocity.mut_replace(motor.getVelocity(), RPM));
                            },
                            this));

            this.armSim = new SingleJointedArmSim(
                    config.gearbox,
                    config.kArmReduction,
                    SingleJointedArmSim.estimateMOI(config.kArmLength, config.kArmMass),
                    config.kArmLength,
                    config.kMinAngle.in(Radians),
                    config.kMaxAngle.in(Radians),
                    true,
                    config.kArmStartingAngle.in(Radians),
                    0.02 / 4096,
                    1.0
            );

            this.arm2d =
                    new Mechanism2d(
                            config.kArmLength * 2,
                            config.kArmLength * 2
                    );

            this.armRoot =
                    arm2d.getRoot("ManipArm Root",
                            config.kArmLength,
                            config.kArmLength);

            this.armMech =
                    armRoot.append(
                            new MechanismLigament2d(
                                    "defaultManipArm",
                                    config.kArmLength,
                                    config.kArmStartingAngle.in(Degrees),
                                    6,
                                    new Color8Bit(Color.kOrange)
                            ));

            // Put Mechanism 2d to dashboard, should be N4T
            // Telemetry enum needs made...
            SmartDashboard.putData("Side View", arm2d);
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
    public void simulationPeriodic() {
        // Set the armSim input, we use volts for this.
        armSim.setInputVoltage(appliedVoltage.in(Volts));

        // Update the arm sim, Standard loop time is 20ms.
        armSim.update(0.02);

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        motor.setPosition(ManipMath.Arm.convertAngleToSensorUnits(armConstants.kArmReduction, Radians.of(armSim.getAngleRads())).in(Rotations));

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        // Update the Mechanism Arm based on simulated arm angle
        armMech.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

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
     */
    public void addAbsoluteEncoderValue(double absEncoderDegrees) {
        absEncoderAngle.mut_replace(absEncoderDegrees, Degrees);
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
     * Set up an arm {@link MechanismLigament2d} for simulation.
     * This is specific to your bot.
     */
    public void overrideArmMech(Mechanism2d arm2d, MechanismRoot2d armRoot, MechanismLigament2d armMech) {
        if (isAdvancedEnabled) {
            this.arm2d = arm2d;
            this.armRoot = armRoot;
            this.armMech = armMech;
        } else {
            DriverStation.reportError("Advanced ManipArm is required for setupArmMech()", true);
        }
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
     * Seeds inbuilt encoder with absolute encoder value.
     * Syncs on init by default.
     */
    public void synchronizeAbsoluteEncoder() {
        motor.setPosition(absEncoderAngle.minus(armConstants.kArmOffsetToHorizantalZero)
                .in(Rotations));
    }

    /**
     * Runs the SysId routine to tune the Arm
     *
     * @return SysId Routine command
     */
    public Command runSysIdRoutine() {
        if (isAdvancedEnabled) {
            return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atMax)
                    .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)).until(atMin)
                    .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)).until(atMax)
                    .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)).until(atMin);
        } else {
            DriverStation.reportWarning("Advanced ManipArm is not setup, for safety SysID is disabled", true);
            return run(() -> {
            });
        }
    }

    /**
     * Run the control loop to reach and maintain the setpoint from the preferences.
     */
    public void reachSetpoint(double setpoint) {
        if (isAdvancedEnabled) {
            limitSwitchFunction();
            double goalPosition = ManipMath.Arm.convertAngleToSensorUnits(armConstants.kArmReduction, Degrees.of(setpoint)).in(Rotations);
            double pidOutput = motor.getRioController().calculate(motor.getPosition(), goalPosition);
            TrapezoidProfile.State setpointState = motor.getRioController().getSetpoint();

            double output = pidOutput +
                    feedforward.calculate(setpointState.position,
                            setpointState.velocity);

            appliedVoltage.mut_replace(Volts.of(output));
            motor.setVoltage(output);

            if (atMin.getAsBoolean() || atMax.getAsBoolean()) {
                stopArm();
            }

        } else {
            limitSwitchFunction();
            motor.setReference(setpoint);
        }
    }

    /**
     * Basic method to run the arm at commanded speed.
     */
    public void runArm(double speed) {
        limitSwitchFunction();
        appliedVoltage.mut_replace(Volts.of(RoboRioSim.getVInVoltage() * speed));
        motor.set(speed);
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
     * Runs runArm as a {@link Command}.
     */
    public Command runArmCommand(double speed) {
        return run(() -> runArm(speed));
    }

    /**
     * Sets the {@link Trigger} for when the top limit switch is hit for {@link ManipArm}.
     *
     * @param topLimitHit top limit switch {@link Trigger}.
     */
    public void setTopLimitSwitch(Trigger topLimitHit) {
        this.topLimitHit = topLimitHit;
    }

    /**
     * Sets the {@link Trigger} for when the bottom limit switch is hit for {@link ManipArm}.
     *
     * @param bottomLimitHit bottom limit switch {@link Trigger}.
     */
    public void setBottomLimitSwitch(Trigger bottomLimitHit) {
        this.bottomLimitHit = bottomLimitHit;
    }

    /**
     * Function that sees if there's active limit switches then stops the {@link ManipArm} if one is hit.
     * Also sets soft limits based off of given Min and Max positions.
     */
    public void limitSwitchFunction() {

        if (atMax.getAsBoolean() || atMin.getAsBoolean()) {
            stopArm();
        }

        if (topLimitHit != null) {
            if (motor.getAppliedOutput() > 0 && bottomLimitHit.getAsBoolean()) {
                stopArm();
            } else {
                Commands.none(); // Stop stopping the arm
            }
        }
        if (bottomLimitHit != null) {
            if (motor.getAppliedOutput() < 0 && bottomLimitHit.getAsBoolean()) {
                stopArm();
            } else {
                Commands.none(); // Stop stopping the arm
            }
        }
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
    public Command toggleAutoStow() {
        return run(() -> {
            defaultCommandOverride = !defaultCommandOverride;
        });
    }

    /**
     * Stops the arm.
     */
    public void stopArm() {
        motor.stopMotor();
        appliedVoltage.mut_replace(Volts.of(0));
    }
}
