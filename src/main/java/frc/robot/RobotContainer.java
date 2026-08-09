// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggerhead;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class RobotContainer {
  public final CustomPivotAngleSource customPivotAngleSource = new CustomPivotAngleSource();

  // public final Drivetrain drive = new Drivetrain();
  public final Intake intake = new Intake();
  public final Pivot pivot = new Pivot();
  public final Flywheel shooter = new Flywheel();
  public final Indexer indexer = new Indexer();

  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    SmartDashboard.putBoolean("is field oriented", false);

    // drive.setDefaultCommand(
    //     drive.driveCommand(
    //         driverController::getLeftY,
    //         driverController::getLeftX,
    //         driverController::getRightX,
    //         () -> SmartDashboard.getBoolean("is field oriented", false)));

    configureBindings();

    // Loggerhead.getInstance().getConfigurator().setConfigureCallback(this::configureLogging);
    configureLogging();
    Loggerhead.getInstance().initializeLogging();
  }

  public void addPeriodic(Robot robot) {
    robot.addPeriodic(Loggerhead.getInstance()::update, 0.02);
  }

  private void configureLogging() {
    Loggerhead.getInstance()
        .getRootTable()
        .getSubTable("pivot")
        .addLoggable(pivot, LogMode.NetworkOnly)
        .getParent()
        .getSubTable("indexer")
        .addLoggable(indexer, LogMode.NetworkOnly)
        .getParent()
        .getSubTable("intake")
        .addLoggable(intake, LogMode.NetworkOnly)
        .getParent()
        .getSubTable("shooter")
        .addLoggable(shooter, LogMode.NetworkOnly);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
