package frc.robot.commands.pivotcommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CustomPivotAngleSource;
import frc.robot.subsystems.Pivot;

public class PivotCommands {
  public static Command setPivotAmp(Pivot pivot) {
    return Commands.run(() -> pivot.setAngle(Degrees.of(103)), pivot);
  }

  public static Command setPivotSpeaker(Pivot pivot) {
    return Commands.run(() -> pivot.setAngle(Degrees.of(-13)), pivot);
  }

  public static Command setPivotIntake(Pivot pivot) {
    return Commands.run(() -> pivot.setAngle(Degrees.of(-100)), pivot);
  }

  public static Command setPivotCustomDashboard(Pivot pivot, CustomPivotAngleSource cpas) {
    return new PivotCustomDashboard(pivot, cpas);
  }
}
