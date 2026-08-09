package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

public class ShooterCommands {
  public static Command enableFlywheels(Flywheel flywheel) {
    return Commands.runEnd(
        () -> flywheel.setDutyCycle(1), () -> flywheel.setDutyCycle(0), flywheel);
  }

  public static Command indexerPush(Indexer indexer) {
    return Commands.runEnd(() -> indexer.setDutyCycle(1), () -> indexer.setDutyCycle(0), indexer);
  }

  public static Command indexerSuck(Indexer indexer) {
    return Commands.runEnd(() -> indexer.setDutyCycle(-1), () -> indexer.setDutyCycle(0), indexer);
  }
}
