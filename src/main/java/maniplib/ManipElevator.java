package maniplib;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import maniplib.encoders.ManipEncoder;
import maniplib.encoders.ManipSparkEncoder;
import maniplib.motors.ManipMotor;
import maniplib.motors.ManipSparkMax;
import maniplib.utils.ManipElevatorConfig;

public class ManipElevator extends SubsystemBase {

  public ManipMotor motor;
  public ManipEncoder encoder;
  public Object controller;
  public DigitalInput topLimitSwitch;
  public DigitalInput bottomLimitSwitch;
  public DCMotor gearbox;
  public ElevatorFeedforward feedforward;
  public double elevatorGearing;
  public double elevatorCarriageMass;
  public double elevatorDrumRadius;
  public ElevatorSim elevatorSim;
  public double rootX = 10;
  public double rootY = 0;
  public double elevatorAngle = 90;
  public Mechanism2d mech2d;
  public MechanismRoot2d mech2dRoot;
  public MechanismLigament2d elevatorMech2d;
  private double maxVelocity = 3.5;
  private double maxAccel = 2.5;

  public ManipElevator(ManipMotor motor, ManipElevatorConfig config) {
    this.motor = motor;
    this.feedforward = config.elevatorFeedforward;
    this.gearbox = config.manipMotorType;
    this.elevatorGearing = config.elevatorGearing;
    this.elevatorDrumRadius = config.elevatorDrumRadius;
    this.elevatorCarriageMass = config.elevatorCarriageMass;

    elevatorSim =
        new ElevatorSim(
            gearbox,
            elevatorGearing,
            elevatorCarriageMass,
            elevatorDrumRadius,
            config.minHeight,
            config.maxHeight,
            true,
            0,
            .01);

    mech2d = new Mechanism2d(config.elevatorWidth, config.elevatorHeight);
    mech2dRoot = mech2d.getRoot("Elevator Root", rootX, rootY);
    elevatorMech2d =
        mech2dRoot.append(
            new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), elevatorAngle));

    if (motor instanceof ManipSparkMax) {

      encoder = new ManipSparkEncoder(motor);

      controller = ((SparkMax) motor.getMotor()).getClosedLoopController();

      SparkMaxConfig sparkConfig = new SparkMaxConfig();
      sparkConfig
          .encoder
          .positionConversionFactor(config.elevatorDrumRadius * 2 * Math.PI) // Rotation to Meters
          .velocityConversionFactor((config.elevatorDrumRadius * 2 * Math.PI) / 60); // RMP to MPS

      sparkConfig.idleMode(IdleMode.kBrake);

      sparkConfig
          .closedLoop
          .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
          .pid(0, 0, 0)
          .maxMotion
          .maxVelocity(maxVelocity)
          .maxAcceleration(maxAccel)
          .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal)
          .allowedClosedLoopError(0.01);

      ((SparkMax) motor.getMotor())
          .configure(
              sparkConfig,
              SparkBase.ResetMode.kNoResetSafeParameters,
              SparkBase.PersistMode.kPersistParameters);

    } else {
      DriverStation.reportWarning("ManipElevator motor not recognized", true);
    }
  }

  public ManipElevator(ManipMotor motor) {
    this.motor = motor;
  }

  public void simulationPeriodic() {
    if (motor instanceof ManipSparkMax) {
      SparkMaxSim motorSim = new SparkMaxSim(((SparkMax) motor.getMotor()), gearbox);

      elevatorSim.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

      elevatorSim.update(0.02);

      motorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);

      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
  }

  public void changeSimConfig(double rootX, double rootY, double elevatorAngle) {
    this.rootX = rootX;
    this.rootY = rootY;
    this.elevatorAngle = elevatorAngle;
  }

  public void changeEncoder(ManipEncoder encoder) {
    this.encoder = encoder;
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public void setMaxVelocity(double maxVelocity) {
    this.maxVelocity = maxVelocity;
  }

  public double getMaxAccel() {
    return maxAccel;
  }

  public void setMaxAccel(double maxAccel) {
    this.maxAccel = maxAccel;
  }

  public Command setSpeed(double speed) {
    return runEnd(
        () -> {
          // elevatorLimitSwitchFunction();
          motor.setSpeed(speed);
        },
        () -> {
          stopCommand();
        });
  }

  public Command setReference(double setpoint, SparkBase.ControlType controlType) {
    return run(
        () -> {
          // elevatorLimitSwitchFunction();
          motor.setReference(setpoint, controlType);
        });
  }

  public void reachGoal(double goal) {
    if (motor instanceof ManipSparkMax) {
      elevatorLimitSwitchFunction();
      ((SparkClosedLoopController) controller)
          .setReference(
              goal,
              SparkBase.ControlType.kPosition,
              ClosedLoopSlot.kSlot0,
              feedforward.calculate(encoder.getVelocity()));
    } else {
      // nothing
    }
  }

  public void setTopLimitSwitch(DigitalInput topLimitSwitch) {
    this.topLimitSwitch = topLimitSwitch;
  }

  public void setBottomLimitSwitch(DigitalInput bottomLimitSwitch) {
    this.bottomLimitSwitch = bottomLimitSwitch;
  }

  public void elevatorLimitSwitchFunction() {
    if (motor.getAppliedOutput() < 0 && topLimitSwitch.get()) {
      motor.stopMotor();
    } else if (motor.getAppliedOutput() > 0 && bottomLimitSwitch.get()) {
      motor.stopMotor();
    } else {
      // Do nothing
    }
  }

  public double getHeight() {
    return encoder.getPosition();
  }

  public Trigger atHeight(double height, double tolerance) {
    return new Trigger(() -> MathUtil.isNear(height, encoder.getPosition(), tolerance));
  }

  public Command setGoal(double goal) {
    return runEnd(
        () -> reachGoal(goal),
        () -> {
          stopCommand();
        });
  }

  public Command stopCommand() {
    return run(
        () -> {
          motor.setSpeed(0);
        });
  }

  public void stop() {
    motor.setSpeed(0.0);
  }

  public void updateTelemetry() {
    // Later
  }

  @Override
  public void periodic() {
    super.periodic();
    updateTelemetry();
  }
}
