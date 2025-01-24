package maniplib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import maniplib.motors.ManipMotor;

public class ManipElevator extends SubsystemBase {

    public ManipMotor motor;

    public DigitalInput topLimitSwitch;
    public DigitalInput bottomLimitSwitch;

    /**
     *  Initialize the {@link ManipElevator} to be used.
     *
     * @param motor motor to set as the lead motor for this {@link ManipElevator}
     */
    public ManipElevator(ManipMotor motor) {
        this.motor = motor;
        motor.setMotorBrake(true);
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
     * A command that moves {@link ManipElevator} at speed.
     *
     * @param speed percent to run {@link  ManipElevator} at.
     * @return a command that moves {@link ManipElevator} at speed.
     */
    public Command setSpeed(double speed) {
        return runEnd(
                () -> {
                    limitSwitchFunction();
                    motor.set(speed);
                },
                motor::stopMotor);
    }

    /**
     * A command that moves {@link ManipElevator} to setpoint.
     *
     * @param setpoint value to move to
     * @return a command that moves {@link ManipElevator} to setpoint.
     */
    public Command setReference(double setpoint) {
        return runEnd(
                () -> {
                    limitSwitchFunction();
                    motor.setReference(setpoint);
                },
                motor::stopMotor);
    }

    /**
     * Stops the {@link ManipElevator}
     */
    public void stopElevator() {
        motor.stopMotor();
    }

    /**
     * A command to stop the {@link ManipElevator}
     *
     * @return a command to stop the {@link ManipElevator}.
     */
    public Command stopElevatorCommand() {
        return motor.stopMotorCommand();
    }

    /**
     * Sets the topLimitSwitch {@link DigitalInput} for {@link ManipElevator}.
     *
     * @param topLimitSwitch top limit switch {@link DigitalInput}.
     */
    public void setTopLimitSwitch(DigitalInput topLimitSwitch) {
        this.topLimitSwitch = topLimitSwitch;
    }

    /**
     * Sets the bottomLimitSwitch {@link DigitalInput} for {@link ManipElevator}.
     *
     * @param bottomLimitSwitch bottom limit switch {@link DigitalInput}.
     */
    public void setBottomLimitSwitch(DigitalInput bottomLimitSwitch) {
        this.bottomLimitSwitch = bottomLimitSwitch;
    }

    /**
     * Function that sees if there's active limit switches then stops the {@link ManipElevator} if one is hit.
     */
    public void limitSwitchFunction() {
        if (topLimitSwitch != null) {
            if (motor.getAppliedOutput() > 0 && topLimitSwitch.get()) {
                motor.stopMotor();
            } else {
                // Stop stopping the motor
            }
        }
        if (bottomLimitSwitch != null) {
            if (motor.getAppliedOutput() < 0 && bottomLimitSwitch.get()) {
                motor.stopMotor();
            } else {
                // Stop stopping the motor
            }
        }
    }
}
