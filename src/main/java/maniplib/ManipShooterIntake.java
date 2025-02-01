package maniplib;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import maniplib.motors.ManipMotor;
import maniplib.utils.PIDControlType;

public class ManipShooterIntake extends SubsystemBase {

    private final ManipMotor motor;

  /**
   *  Initialize the {@link ManipShooterIntake} to be used.
   *
   * @param motor motor to set as the lead motor for this {@link ManipShooterIntake}
   */
    public ManipShooterIntake(ManipMotor motor) {
        this.motor = motor;
        motor.setPIDControlType(PIDControlType.ControlType.VELOCITY);
    }

    /**
     * Sets the {@link ManipMotor} to follow another {@link ManipMotor}.
     *
     * @param followerMotor {@link ManipMotor} to follow the lead motor.
     * @param isInverted whether to invert the follower or not.
     */
    public void addFollower(ManipMotor followerMotor, boolean isInverted) {
      followerMotor.setAsFollower(motor, isInverted);
    }

  /**
   * Set the percentage output.
   *
   * @param speed percent out for the motor controller.
   */
  public Command setSpeed(double speed) {
        return runEnd(
                () -> {
                    motor.set(speed);
                },
                motor::stopMotor);
    }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint, value type changes with ControlType.
   * @param controlType ControlType to run the setReference as.
   */
    public Command setReference(double setpoint, SparkBase.ControlType controlType) {
        return runEnd(
                () -> {
                    motor.setReference(setpoint);
                },
                motor::stopMotor
        );
    }

  /**
   * Stop the {@link ManipShooterIntake}
   */
  public void stopShooter() {
    motor.stopMotor();
  }

  /**
   * A command that stops the {@link ManipShooterIntake}
   *
   * @return a command that stops the {@link ManipShooterIntake}
   */
  public Command stopShooterCommand() {
        return motor.stopMotorCommand();
    }

}
