package maniplib.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.*;

public class ManipArmConstants {

    public final DCMotor gearbox;

    // The P gain for the PID controller that drives this arm.
    public final double kArmKp;
    public final double kArmKi;
    public final double kArmKd;

    public final double kArmkS; // volts (V)
    public final double kArmkG; // volts (V)
    public final double kArmKv; // volts per velocity (V/RPM)
    public final double kArmKa; // volts per acceleration (V/(RPM/s))

    public final Angle kArmAllowedClosedLoopError;
    public final double kArmReduction;
    public final double kArmMass; // Kilograms
    public final double kArmLength;
    public final Angle kArmStartingAngle;
    public final Angle kMinAngle;
    public final Angle kMaxAngle;
    public final double kArmRampRate;
    public final Angle kArmOffsetToHorizantalZero;
    public final boolean kArmInverted;

    public final double kArmMaxVelocityRPM;
    public final double kArmMaxAccelerationRPMperSecond;
    public final int kArmStallCurrentLimitAmps;

    // Toggle for enabling advanced features, false will just use basic PID control.
    public boolean kEnableAdvanced = false;

    /**
     * Sets the constant values for {@link maniplib.ManipArm}.
     *
     * @param gearbox                         DCMotor used to determine how many of which motor is being used.
     * @param kArmKp                          PID kP Tuning Value.
     * @param kArmKi                          PID kI Tuning Value.
     * @param kArmKd                          PID kD Tuning Value.
     * @param kArmkS                          FeedForward kS Tuning Value. volts(V)
     * @param kArmkG                          FeedForward kV Tuning Value. volt per velocity (V/(m/s))
     * @param kArmkV                          FeedForward kA Tuning Value. volt per acceleration (V/(m/s²))
     * @param kArmkA                          FeedForward kA Tuning Value. volts(V)
     * @param kArmReduction                   Gear ratio of the arm, use gearbox and sprockets.
     * @param kArmMassLbs                     How much the arm weighs in pounds.
     * @param kArmLengthInches                How long the arm is in inches. Used in sim.
     * @param kArmStartingAngle               Where the arm sim should set the arm on start.
     * @param kMinAngle                       Arms max height in degrees. Used for soft limits as well.
     * @param kMaxAngle                       Arms min height in degrees. Used for soft limits as well.
     * @param kArmInverted                    Whether to invert the drive direction of the arm. runArm(.1); should go up.
     * @param kArmRampRate                    Elevators ramp rate. 0.5 is recommended for most.
     * @param kArmOffsetToHorizantalZero      Absolute encoder offset. Arm should be horizontal to the floor at 0.
     * @param kArmAllowedClosedLoopError      Allowed error in the pid control in degrees.
     * @param kArmStallCurrentLimitAmps       The arms stall limit. 30 is recommended for most.
     * @param kArmMaxVelocityRPM              Arms max velocity in rotations per second.
     * @param kArmMaxAccelerationRPMperSecond Arms max Acceleration in rotations per second. Depends on specific arm config for accurate sim speed.
     * @param kEnableAdvanced                 Determines whether to use advanced control and sim.
     */
    public ManipArmConstants(
            DCMotor gearbox,
            double kArmKp,
            double kArmKi,
            double kArmKd,
            double kArmkS,
            double kArmkG,
            double kArmkV,
            double kArmkA,
            double kArmReduction,
            double kArmMassLbs,
            double kArmLengthInches,
            double kArmStartingAngle,
            double kMinAngle,
            double kMaxAngle,
            boolean kArmInverted,
            double kArmRampRate,
            double kArmOffsetToHorizantalZero,
            double kArmAllowedClosedLoopError,
            int kArmStallCurrentLimitAmps,
            double kArmMaxVelocityRPM,
            double kArmMaxAccelerationRPMperSecond,
            boolean kEnableAdvanced
    ) {
        this.gearbox = gearbox;
        this.kArmKp = kArmKp;
        this.kArmKi = kArmKi;
        this.kArmKd = kArmKd;
        this.kArmkS = kArmkS;
        this.kArmkG = kArmkG;
        this.kArmKv = kArmkV;
        this.kArmKa = kArmkA;
        this.kArmReduction = kArmReduction;
        this.kArmMass = Kilograms.convertFrom(kArmMassLbs, Pounds);
        this.kArmLength = Inches.of(kArmLengthInches).in(Meters);
        this.kArmStartingAngle = Degrees.of(kArmStartingAngle);
        this.kMinAngle = Degrees.of(kMinAngle);
        this.kMaxAngle = Degrees.of(kMaxAngle);
        this.kArmInverted = kArmInverted;
        this.kArmRampRate = kArmRampRate;
        this.kArmOffsetToHorizantalZero = Degrees.of(kArmOffsetToHorizantalZero);
        this.kArmAllowedClosedLoopError = ManipMath.Arm.convertAngleToSensorUnits(kArmReduction, Degrees.of(kArmAllowedClosedLoopError));
        this.kArmStallCurrentLimitAmps = kArmStallCurrentLimitAmps;
        this.kArmMaxVelocityRPM = ManipMath.Arm.convertAngleToSensorUnits(kArmReduction, Degrees.of(kArmMaxVelocityRPM)).per(
                Second).in(RPM);
        this.kArmMaxAccelerationRPMperSecond = ManipMath.Arm.convertAngleToSensorUnits(kArmReduction, Degrees.of(kArmMaxAccelerationRPMperSecond)).per(
                        Second).per(Second)
                .in(RPM.per(Second));
        this.kEnableAdvanced = kEnableAdvanced;
    }
}
