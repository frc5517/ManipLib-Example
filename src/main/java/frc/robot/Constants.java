package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import maniplib.utils.ManipArmConstants;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.ArmConstants.armConfig;

public class Constants {

    public static final Mechanism2d sideView = new Mechanism2d(armConfig.kArmLength * 2, 1 + armConfig.kArmLength);

    public static final class ArmConstants {
        public static final double armSpeed = 0.15;
        public static final double armSetpoint = 5;

        public static final ManipArmConstants armConfig =
                new ManipArmConstants(
                        DCMotor.getNEO(1),
                        2.354,
                        0,
                        0,
                        0,
                        0, // 60:1 with 12:28 sprockets.
                        .2,
                        0.0683,
                        140,
                        3,
                        17,
                        0,
                        -90,
                        90,
                        false,
                        0.5,
                        30,
                        0.01,
                        30,
                        5,
                        5,
                        true
                );
    }

    public static final class ElevatorConstants {
        public static final double elevatorSpeed = .3;
        public static final double elevatorSetpoint = 10;
    }

    public static final class IntakeShooterConstants {
        public static final double intakeSpeed = .3;
        public static final double shootSpeed = .5;
        public static final double velocitySetpoint = 100;
    }

}
