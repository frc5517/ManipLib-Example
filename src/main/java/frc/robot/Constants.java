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
                        0.01,
                        140, // 60:1 with 12:28 sprockets.
                        3,
                        17,
                        45,
                        -90,
                        90,
                        0.5,
                        0,
                        false,
                        5,
                        5,
                        30,
                        0,
                        0,
                        0.2,
                        0.0683,
                        true
                );

        public static final MechanismRoot2d armBase =
                sideView.getRoot("Arm Base",
                        .5,
                        1);

        public static final MechanismLigament2d armMech =
                armBase.append(
                        new MechanismLigament2d(
                                "defaultManipArm",
                                armConfig.kArmLength,
                                armConfig.kArmStartingAngle.in(Degrees),
                                6,
                                new Color8Bit(Color.kRed)
                        ));


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
