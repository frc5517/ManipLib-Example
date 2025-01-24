package maniplib;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import maniplib.motors.ManipMotor;
import maniplib.motors.ManipSparkMax;

public class ManipShooter extends SubsystemBase {

  private final ManipMotor motor;

  private boolean isBrakeMode = true;

  public ManipShooter(ManipMotor motor) {
    this.motor = motor;

    if (motor instanceof ManipSparkMax) {
      SparkMaxConfig sparkConfig = new SparkMaxConfig();

      if (isBrakeMode) {
        sparkConfig.idleMode(IdleMode.kBrake);
      }

      ((ManipSparkMax) motor).updateConfig(sparkConfig);
    }
  }

  public Command setSpeed(double speed) {
    return runEnd(
        () -> {
          motor.setSpeed(speed);
        },
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
    motor.setSpeed(0);
  }

  public void setBrakeMode(boolean isBrakeMode) {
    this.isBrakeMode = isBrakeMode;
  }
}
