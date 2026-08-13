package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.pivotcommands.PivotCommands;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class ShootSequence extends ParallelCommandGroup {
  private boolean noteHasLeft = false;
  private boolean pivotAtSetpoint = false;

  public ShootSequence(Intake intake, Flywheel flywheel, Pivot pivot, Indexer indexer) {
    noteHasLeft = false;
    pivotAtSetpoint = false;

    addCommands(
        Commands.runEnd(() -> flywheel.setDutyCycle(1), flywheel::setOff, flywheel)
            .until(() -> noteHasLeft),
        Commands.sequence(
            PivotCommands.setPivotSpeaker(pivot)
                .alongWith(Commands.run(() -> pivotAtSetpoint = pivot.isAtSetpoint.getAsBoolean()))
                .until(() -> noteHasLeft),
            PivotCommands.setPivotIntake(pivot)),
        Commands.sequence(
            new WaitUntilCommand(() -> pivotAtSetpoint),
            Commands.runEnd(() -> indexer.setDutyCycle(1), indexer::setOff, indexer)
                .until(indexer.noteSensorActive.negate()),
            new WaitUntilCommand(1),
            Commands.runOnce(() -> noteHasLeft = true)));
  }
}
