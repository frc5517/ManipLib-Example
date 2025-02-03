package maniplib.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.*;

public class ManipArmConstants {

    public final DCMotor gearbox;

    // The P gain for the PID controller that drives this arm.
    public final double kArmKp;
    public final double kArmKi;
    public final double kArmKd;
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

    public final double kArmkS; // volts (V)
    public final double kArmkG; // volts (V)
    public final double kArmKv; // volts per velocity (V/RPM)
    public final double kArmKa; // volts per acceleration (V/(RPM/s))

    // Toggle for enabling advanced features, false will just use basic PID control.
    public boolean kEnableAdvanced = false;

    public ManipArmConstants(
            DCMotor gearbox,
            double kArmKp,
            double kArmKi,
            double kArmKd,
            double kArmAllowedClosedLoopError,
            double kArmReduction,
            double kArmMass,
            double kArmLength,
            double kArmStartingAngle,
            double kMinAngle,
            double kMaxAngle,
            double kArmRampRate,
            double kArmOffsetToHorizantalZero,
            boolean kArmInverted,
            double kArmMaxVelocityRPM,
            double kArmMaxAccelerationRPMperSecond,
            int kArmStallCurrentLimitAmps,
            double kArmkS,
            double kArmkG,
            double kArmKv,
            double kArmKa,
            boolean kEnableAdvanced
    ) {
        this.gearbox = gearbox;
        this.kArmKp = kArmKp;
        this.kArmKi = kArmKi;
        this.kArmKd = kArmKd;
        this.kArmAllowedClosedLoopError = ManipMath.Arm.convertAngleToSensorUnits(kArmReduction, Degrees.of(kArmAllowedClosedLoopError));
        this.kArmReduction = kArmReduction;
        this.kArmMass = kArmMass;
        this.kArmLength = Inches.of(kArmLength).in(Meters);
        this.kArmStartingAngle = Radians.of(kArmStartingAngle);
        this.kMinAngle = Radians.of(Units.degreesToRadians(kMinAngle));
        this.kMaxAngle = Radians.of(Units.degreesToRadians(kMaxAngle));
        this.kArmRampRate = kArmRampRate;
        this.kArmOffsetToHorizantalZero = Rotations.of(kArmOffsetToHorizantalZero);
        this.kArmInverted = kArmInverted;
        this.kArmMaxVelocityRPM = ManipMath.Arm.convertAngleToSensorUnits(kArmReduction, Degrees.of(kArmMaxVelocityRPM)).per(
                Second).in(RPM);
        this.kArmMaxAccelerationRPMperSecond = ManipMath.Arm.convertAngleToSensorUnits(kArmReduction, Degrees.of(kArmMaxAccelerationRPMperSecond)).per(
                        Second).per(Second)
                .in(RPM.per(Second));
        this.kArmStallCurrentLimitAmps = kArmStallCurrentLimitAmps;
        this.kArmkS = kArmkS;
        this.kArmkG = kArmkG;
        this.kArmKv = kArmKv;
        this.kArmKa = kArmKa;
        this.kEnableAdvanced = kEnableAdvanced;
    }
}
