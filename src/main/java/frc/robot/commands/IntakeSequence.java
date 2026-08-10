package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.pivotcommands.PivotCommands;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class IntakeSequence extends ParallelRaceGroup {
  public IntakeSequence(Intake intake, Flywheel flywheel, Pivot pivot, Indexer indexer) {
    super(
        Commands.parallel(
            Commands.runEnd(intake::setOn, intake::setOff, intake),
            Commands.runEnd(() -> flywheel.setDutyCycle(0.1), flywheel::setOff, flywheel),
            Commands.runEnd(() -> indexer.setDutyCycle(1), indexer::setOff, indexer),
            PivotCommands.setPivotIntake(pivot)),
        new WaitUntilCommand(indexer.noteSensorActive));

    addRequirements(intake, flywheel, pivot, indexer);
  }
}
