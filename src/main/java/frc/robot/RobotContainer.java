// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.compoundlogger.LogSubsystemCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AmpIntakeSequence;
import frc.robot.commands.ShootSequence;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class RobotContainer {
  public final CustomPivotAngleSource customPivotAngleSource = new CustomPivotAngleSource();

  // public final Drivetrain drive = new Drivetrain();
  public final Intake intake = new Intake();
  public final Pivot pivot = new Pivot();
  public final Flywheel flywheel = new Flywheel();
  public final Indexer indexer = new Indexer();

  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    // drive.setDefaultCommand(
    //     drive.driveCommand(
    //         driverController::getLeftY,
    //         driverController::getLeftX,
    //         driverController::getRightX,
    //         () -> SmartDashboard.getBoolean("is field oriented", false)));

    configureBindings();

    Loggerhead.getInstance().getConfigurator().setConfigureCallback(this::configureLogging);
    Loggerhead.getInstance().initializeLogging();
  }

  public void addPeriodic(Robot robot) {
    robot.addPeriodic(Loggerhead.getInstance()::update, 0.02);
  }

  private void configureLogging() {
    var lm = LogMode.NetworkOnly;
    Loggerhead.getInstance()
        .getRootTable()
        .getSubTable("pivot")
        .addLoggable(pivot, lm)
        .addCompoundLogger(new LogSubsystemCommands("Commands", lm, pivot))
        .getParent()
        .getSubTable("indexer")
        .addLoggable(indexer, lm)
        .addCompoundLogger(new LogSubsystemCommands("Commands", lm, indexer))
        .getParent()
        .getSubTable("intake")
        .addLoggable(intake, lm)
        .addCompoundLogger(new LogSubsystemCommands("Commands", lm, intake))
        .getParent()
        .getSubTable("shooter")
        .addLoggable(flywheel, lm)
        .addCompoundLogger(new LogSubsystemCommands("Commands", lm, flywheel));
  }

  private void configureBindings() {
    // driverController.a().onTrue(new IntakeSequence(intake, flywheel, pivot, indexer));
    // driverController.b().onTrue(PivotCommands.setPivotSpeaker(pivot));
    // driverController.x().onTrue(new ShootSequence(intake, flywheel, pivot, indexer));
    driverController.x().onTrue(new AmpIntakeSequence(intake, flywheel, pivot, indexer));
    driverController.y().onTrue(new ShootSequence(intake, flywheel, pivot, indexer));
    // driverController.a().onTrue(PivotCommands.setPivotSpeaker(pivot));
    // driverController.b().onTrue(PivotCommands.setPivotAmp(pivot));
    // driverController.x().onTrue(PivotCommands.setPivotIntake(pivot));

    // driverController
    //     .y()
    //     .whileTrue(Commands.runEnd(() -> flywheel.setDutyCycle(1), flywheel::setOff, flywheel));

    // driverController
    //     .leftBumper()
    //     .whileTrue(Commands.runEnd(() -> indexer.setDutyCycle(1), indexer::setOff, indexer));

    // driverController
    //     .rightBumper()
    //     .whileTrue(Commands.runEnd(() -> intake.setDutyCycle(1), intake::setOff, intake));
    // driverController.b().onTrue()
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
