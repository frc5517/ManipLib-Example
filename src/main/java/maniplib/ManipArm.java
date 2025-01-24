package maniplib;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import maniplib.encoders.ManipEncoder;
import maniplib.encoders.ManipSparkEncoder;
import maniplib.motors.ManipMotor;
import maniplib.motors.ManipSparkMax;

public class ManipArm extends SubsystemBase {

  public ManipMotor motor;

  public ManipEncoder encoder;

  public Object controller;

  public DigitalInput topLimitSwitch;
  public DigitalInput bottomLimitSwitch;

  public double maxVelocity = 2;
  public double maxAccel = 1.5;

  public ManipArm(ManipMotor motor) {
    this.motor = motor;

    if (motor instanceof ManipSparkMax) {

      motor.setBrakeMode(true);

      encoder = new ManipSparkEncoder(motor);

      controller = ((SparkMax) motor.getMotor()).getClosedLoopController();

      SparkMaxConfig sparkConfig = new SparkMaxConfig();

      /*
      sparkConfig.encoder
              .positionConversionFactor(config.elevatorDrumRadius * 2 * Math.PI) // Rotation to Meters
              .velocityConversionFactor((config.elevatorDrumRadius * 2 * Math.PI) / 60); // RMP to MPS
              */

      sparkConfig
          .closedLoop
          .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
          .pid(0, 0, 0)
          .maxMotion
          .maxVelocity(maxVelocity)
          .maxAcceleration(maxAccel)
          .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal)
          .allowedClosedLoopError(0.01);

      ((ManipSparkMax) motor.getMotor()).updateConfig(sparkConfig);

    } else {
      DriverStation.reportWarning("ManipElevator motor not recognized", true);
    }
  }

  public Command setSpeed(double speed) {
    return runEnd(
        () -> {
          // limitSwitchFunction();
          motor.setSpeed(speed);
        },
        () -> {
          stopArm();
        });
  }

  public Command setReference(double setpoint, SparkBase.ControlType controlType) {
    return runEnd(
        () -> {
          // limitSwitchFunction();
          motor.setReference(setpoint, controlType);
        },
        () -> {
          stopArm();
        });
  }

  public Command stopArm() {
    return run(
        () -> {
          motor.setSpeed(0.0);
        });
  }

  public void setTopLimitSwitch(DigitalInput topLimitSwitch) {
    this.topLimitSwitch = topLimitSwitch;
  }

  public void setBottomLimitSwitch(DigitalInput bottomLimitSwitch) {
    this.bottomLimitSwitch = bottomLimitSwitch;
  }

  public void limitSwitchFunction() {
    if (motor.getAppliedOutput() > 0 && topLimitSwitch.get()) {
      motor.stopMotor();
    } else if (motor.getAppliedOutput() < 0 && bottomLimitSwitch.get()) {
      motor.stopMotor();
    } else {
      // Stop stopping the motor
    }
  }
}
