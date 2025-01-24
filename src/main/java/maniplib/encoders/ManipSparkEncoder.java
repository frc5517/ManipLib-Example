package maniplib.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import maniplib.motors.ManipMotor;
import maniplib.motors.ManipSparkMax;

/** SparkMax absolute encoder, attached through the data port. */
public class ManipSparkEncoder extends ManipEncoder {

  /** The {@link AbsoluteEncoder} representing the duty cycle encoder attached to the SparkMax. */
  public RelativeEncoder encoder;
  /** An {@link Alert} for if there is a failure configuring the encoder. */
  @SuppressWarnings("unused")
  private final Alert failureConfiguring;
  /** An {@link Alert} for if there is a failure configuring the encoder offset. */
  @SuppressWarnings("unused")
  private final Alert offsetFailure;

  private final ManipMotor sparkMax;

  /**
   * Create the {@link ManipSparkEncoder} object as a duty cycle from the {@link SparkMax} motor.
   *
   * @param motor Motor to create the encoder from.
   */
  public ManipSparkEncoder(ManipMotor motor) {
    failureConfiguring =
        new Alert("Encoders", "Failure configuring SparkMax Encoder", AlertType.kWarning);
    offsetFailure =
        new Alert("Encoders", "Failure to set Absolute Encoder Offset", AlertType.kWarning);
    if (motor.getMotor() instanceof SparkMax) {
      sparkMax = motor;
      encoder = ((SparkMax) motor.getMotor()).getEncoder();
      motor.setEncoder(this);
    } else {
      throw new RuntimeException("Motor given to instantiate SparkMaxEncoder is not a CANSparkMax");
    }
  }

  /** Reset the encoder to factory defaults. */
  @Override
  public void factoryDefault() {
    // Do nothing
  }

  /** Clear sticky faults on the encoder. */
  @Override
  public void clearStickyFaults() {
    // Do nothing
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  @Override
  public Object getEncoder() {
    return encoder;
  }

  /**
   * Sets the Absolute Encoder Offset inside the SparkMax's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset) {
    if (sparkMax instanceof ManipSparkMax) {
      SparkMaxConfig cfg = ((ManipSparkMax) sparkMax).getConfig();
      cfg.absoluteEncoder.zeroOffset(offset);
      ((ManipSparkMax) sparkMax).updateConfig(cfg);
      return true;
    }
    //    } else if (sparkMax instanceof SparkMaxBrushedMotorSwerve)
    //    {
    //      SparkMaxConfig cfg = ((SparkMaxBrushedMotorSwerve) sparkMax).getConfig();
    //      cfg.absoluteEncoder.zeroOffset(offset);
    //      ((SparkMaxBrushedMotorSwerve) sparkMax).updateConfig(cfg);
    //      return true;
    //    }
    return false;
  }

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }
}
