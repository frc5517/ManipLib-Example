package maniplib.motors;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import maniplib.encoders.ManipEncoder;
import maniplib.utils.PIDFConfig;

public class ManipSparkMax extends ManipMotor {

  /** Config retry delay. */
  private final double configDelay = Milliseconds.of(5).in(Seconds);

  private final SparkMax motor;

  public RelativeEncoder encoder;

  public SparkClosedLoopController pid;

  private final SparkMaxConfig cfg = new SparkMaxConfig();

  public ManipSparkMax(int canid, SparkLowLevel.MotorType motorType) {
    this.motor = new SparkMax(canid, motorType);

    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    cfg.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   */
  private void configureSparkMax(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
      Timer.delay(configDelay);
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  /**
   * Get the current configuration of the {@link SparkMax}
   *
   * @return {@link SparkMaxConfig}
   */
  public SparkMaxConfig getConfig() {
    return cfg;
  }

  /**
   * Update the config for the {@link SparkMax}
   *
   * @param cfgGiven Given {@link SparkMaxConfig} which should have minimal modifications.
   */
  public void updateConfig(SparkMaxConfig cfgGiven) {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException(
          "Configuration changes cannot be applied while the robot is enabled.");
    }
    cfg.apply(cfgGiven);
    configureSparkMax(
        () ->
            motor.configure(
                cfg,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  @Override
  public void factoryDefaults() {
    // Do nothing
  }

  @Override
  public void clearStickyFaults() {
    motor.clearFaults();
  }

  @Override
  public ManipEncoder setEncoder(ManipEncoder encoder) {
    return null; // Do nothing
  }

  @Override
  public void setConversionFactor(double conversionFactor) {

    // Nothing for now
  }

  @Override
  public void configurePIDF(PIDFConfig config) {

    cfg.closedLoop
        .pidf(config.p, config.i, config.d, config.f)
        .iZone(config.iz)
        .outputRange(config.output.min, config.output.max);
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    cfg.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(minInput, maxInput);
  }

  @Override
  public void setBrakeMode(boolean isBrakeMode) {
    cfg.idleMode(isBrakeMode ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
  }

  @Override
  public void setInverted(boolean inverted) {
    cfg.inverted(inverted);
  }

  @Override
  public void burnFlash() {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException("Config updates cannot be applied while the robot is Enabled!");
    }
    configureSparkMax(
        () -> {
          return motor.configure(
              cfg,
              SparkBase.ResetMode.kNoResetSafeParameters,
              SparkBase.PersistMode.kPersistParameters);
        });
  }

  @Override
  public SparkMax getMotor() {
    return motor;
  }

  @Override
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  @Override
  public void stopMotor() {
    motor.set(0.0);
  }

  @Override
  public void setSpeed(double percentOutput) {
    motor.set(percentOutput);
  }

  @Override
  public void setReference(double setpoint, SparkBase.ControlType controlType) {
    configureSparkMax(
        () -> pid.setReference(setpoint, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0));
  }
}
