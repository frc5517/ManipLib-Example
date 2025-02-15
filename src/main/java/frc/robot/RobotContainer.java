// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import maniplib.ManipElevator;
import maniplib.ManipArm;
import maniplib.Telemetry;
import maniplib.motors.ManipSparkMax;

import static edu.wpi.first.units.Units.Volts;

public class RobotContainer {

    // Define the ManipLib customized subsystems.
    private final ManipSparkMax armMotor = new ManipSparkMax(1);
    private final ManipArm arm = new ManipArm(armMotor, Constants.ArmConstants.armConfig);

    private final ManipSparkMax leftElevatorMotor = new ManipSparkMax(2);
    private final ManipElevator elevator = new ManipElevator(leftElevatorMotor, Constants.ElevatorConstants.elevatorConfig);

    // We only need operator controller for this project
    private final CommandXboxController operatorController = new CommandXboxController(0);

    public RobotContainer() {

        Telemetry.manipVerbosity = Telemetry.ManipTelemetry.NONE;

        DriverStation.silenceJoystickConnectionWarning(true);

        ManipSparkMax rightElevatorMotor = new ManipSparkMax(3);
        elevator.addFollower(rightElevatorMotor, true);

        SmartDashboard.putData("Side View", Constants.sideRobotView);

        arm.setDefaultCommand(arm.autoStowWithOverride(0));

        elevator.setDefaultCommand(elevator.autoStowWithOverride(0));

        configureBindings();
    }

    private void configureBindings() {

        // Can also make actual subsystem classes with something like armSubsystem.intake() to clean up RobotContainer
        operatorController.a().whileTrue(arm.runEnd(() -> arm.runArmSpeed(1), arm::stopArm));
        operatorController.b().whileTrue(arm.runEnd(() -> arm.runArmSpeed(-1), arm::stopArm));
        operatorController.a().whileTrue(elevator.runEnd(() -> elevator.runElevatorSpeed(1), elevator::stopElevator));
        operatorController.b().whileTrue(elevator.runEnd(() -> elevator.runElevatorSpeed(-1), elevator::stopElevator));

        operatorController.back().onTrue(arm.toggleAutoStow());
        operatorController.back().onFalse(elevator.toggleAutoStow());

        operatorController.y().whileTrue(elevator.setGoal(40));
        operatorController.x().whileTrue(elevator.setGoal(10));
        operatorController.y().whileTrue(arm.setGoal(-75));
        operatorController.x().whileTrue(arm.setGoal(75));

        //operatorController.start().onTrue(arm.runSysIdRoutine());
        operatorController.start().onTrue(elevator.runSysIdRoutine());
    }

    /**
     * Used to update mechanism sims. Called in {@link Robot}.
     */
    public void updateMechSim() {
        Constants.kElevatorCarriage.setPosition(Constants.ArmConstants.armConfig.kArmLength, elevator.getMechLength());
        Constants.kElevatorTower.setLength(elevator.getMechLength());
        Constants.kArmMech.setAngle(arm.getMechAngle());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
