package maniplib.motors;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.system.plant.DCMotor;
import maniplib.encoders.ManipEncoder;
import maniplib.utils.PIDFConfig;

public abstract class ManipMotor {

  /**
   * The maximum amount of times the swerve motor will attempt to configure a motor if failures
   * occur.
   */
  public final int maximumRetries = 5;

  public DCMotor simMotor;

  public abstract void factoryDefaults();

  public abstract void clearStickyFaults();

  public abstract ManipEncoder setEncoder(ManipEncoder encoder);

  public abstract void configurePIDF(PIDFConfig config);

  public abstract void configurePIDWrapping(double minInput, double maxInput);

  public abstract void setBrakeMode(boolean isBrakeMode);

  public abstract void setInverted(boolean inverted);

  public abstract void burnFlash();

  public abstract Object getMotor();

  public abstract double getAppliedOutput();

  public abstract void stopMotor();

  public abstract void setSpeed(double speed);

  public abstract void setReference(double setpoint);
}
