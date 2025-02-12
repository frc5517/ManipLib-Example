package maniplib.utils;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class ManipElevatorConstants {

    public final DCMotor gearbox;
    // PID Tuning values
    public final double kElevatorKp;
    public final double kElevatorKi;
    public final double kElevatorKd;
    // FeedForward Values
    public final double kElevatorkS; // volts(V)
    public final double kElevatorkV; // volt per velocity (V/(m/s))
    public final double kElevatorkA; // volt per acceleration (V/(m/s²))
    public final double kElevatorkG; // volts (V)
    // Sim values for calculations
    public final double kElevatorGearing;
    public final double kElevatorDrumRadius;
    public final double kElevatorCarriageMass;
    // Elevator movement constraints
    public final Distance kStartingHeightSim;
    public final Distance kMaxHeight;
    public final Distance kMinHeight;
    // Elevator speed constraints
    public final double kElevatorRampRate;
    public final int kElevatorCurrentLimit;
    public final double kMaxVelocity;
    public final double kMaxAcceleration;
    // Offset for an optional but heavily recommended abs encoder
    public final double kAbsEncoderOffset;
    // Whether to use basic or advanced controls and sim.
    public final boolean kEnableAdvanced;

    /**
     * Sets the constant values for {@link maniplib.ManipElevator}.
     *
     * @param gearbox DCMotor used to determine how many of which motor is being used.
     * @param kElevatorKp PID kP Tuning Value.
     * @param kElevatorKi PID kI Tuning Value.
     * @param kElevatorKd PID kD Tuning Value.
     * @param kElevatorkS FeedForward kS Tuning Value. volts(V)
     * @param kElevatorkV FeedForward kV Tuning Value. volt per velocity (V/(m/s))
     * @param kElevatorkA FeedForward kA Tuning Value. volt per acceleration (V/(m/s²))
     * @param kElevatorkG FeedForward kG Tuning Value. volts (V)
     * @param kElevatorGearing Gear ratio of the elevator, use gearbox w/o sprockets.
     * @param kElevatorDrumRadiusInches Radius of the drum, sprocket radius if using chain.
     * @param kElevatorCarriageMassLbs How much the carriage weighs in pounds.
     * @param kStartingSimHeightInches Where the elevator sim should set the elevator on start, in inches.
     * @param kMaxHeightInches Elevators max height in inches. Used for soft limits as well.
     * @param kMinHeightInches Elevators min height in inches. Used for soft limits as well.
     * @param kElevatorRampRate Elevators ramp rate. 0.1 is recommended for most.
     * @param kElevatorCurrentLimit Elevators current limit. 40 is recommended for most.
     * @param kMaxVelocityMps Elevators max velocity in meters per second.
     * @param kMaxAccelerationMps Elevators max acceleration in meters per second.
     * @param kAbsEncoderOffset Offset for an optional but heavily recommended abs encoder. Set elevator to kMinHeight then input the raw abs output.
     * @param kEnableAdvanced Determines whether to use advanced control and sim.
     */
    public ManipElevatorConstants(
            DCMotor gearbox,
            double kElevatorKp,
            double kElevatorKi,
            double kElevatorKd,
            double kElevatorkS,
            double kElevatorkV,
            double kElevatorkA,
            double kElevatorkG,
            double kElevatorGearing,
            double kElevatorDrumRadiusInches,
            double kElevatorCarriageMassLbs,
            double kStartingSimHeightInches,
            double kMaxHeightInches,
            double kMinHeightInches,
            double kElevatorRampRate,
            int kElevatorCurrentLimit,
            double kMaxVelocityMps,
            double kMaxAccelerationMps,
            double kAbsEncoderOffset,
            boolean kEnableAdvanced
            ) {
        this.gearbox = gearbox;
        this.kElevatorKp = kElevatorKp;
        this.kElevatorKi = kElevatorKi;
        this.kElevatorKd = kElevatorKd;
        this.kElevatorkS = kElevatorkS;
        this.kElevatorkV = kElevatorkV;
        this.kElevatorkA = kElevatorkA;
        this.kElevatorkG = kElevatorkG;
        this.kElevatorGearing = kElevatorGearing;
        this.kElevatorDrumRadius = Meters.convertFrom(kElevatorDrumRadiusInches, Inches); // // Convert inches to meter double.
        this.kElevatorCarriageMass = Kilograms.convertFrom(kElevatorCarriageMassLbs, Pounds);
        this.kStartingHeightSim = Meters.of(Meters.convertFrom(kStartingSimHeightInches, Inches)); // Convert inches to meter units.
        this.kMaxHeight = Meters.of(Meters.convertFrom(kMaxHeightInches, Inches)); // Convert inches to meter units.
        this.kMinHeight = Meters.of(Meters.convertFrom(kMinHeightInches, Inches)); // Convert inches to meter units.
        this.kElevatorRampRate = kElevatorRampRate;
        this.kElevatorCurrentLimit = kElevatorCurrentLimit;
        this.kMaxVelocity = Meters.of(kMaxVelocityMps).per(Second).in(MetersPerSecond);
        this.kMaxAcceleration = Meters.of(kMaxAccelerationMps).per(Second).per(Second).in(MetersPerSecondPerSecond);
        this.kAbsEncoderOffset = kAbsEncoderOffset;
        this.kEnableAdvanced = kEnableAdvanced;
    }
}
