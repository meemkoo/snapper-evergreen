package frc.robot.commands.pivotcommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CustomPivotAngleSource;
import frc.robot.subsystems.Pivot;

public class PivotCustomDashboard extends Command {
  private final Pivot pivot;
  private final CustomPivotAngleSource cpas;

  private Angle angle = Degrees.of(0);

  public PivotCustomDashboard(Pivot pivot, CustomPivotAngleSource cpas) {
    this.pivot = pivot;
    this.cpas = cpas;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {
    angle = cpas.getAngle();
  }

  @Override
  public void execute() {
    pivot.setAngle(angle);
  }

  @Override
  public void end(boolean inturupted) {}
}
