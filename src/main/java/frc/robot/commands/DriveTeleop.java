package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class DriveTeleop extends Command {
  private final Drivetrain drivetrain;

  private final Supplier<Double> xpercent;
  private final Supplier<Double> ypercent;
  private final Supplier<Double> tpercent;
  private final Supplier<Double> gasPedal;

  public DriveTeleop(
      Drivetrain drivetrain,
      Supplier<Double> xpercent,
      Supplier<Double> ypercent,
      Supplier<Double> tpercent,
      Supplier<Double> gasPedal) {
    this.drivetrain = drivetrain;

    this.gasPedal = gasPedal;
    this.xpercent = xpercent;
    this.ypercent = ypercent;
    this.tpercent = tpercent;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var speeds =
        new ChassisSpeeds(
            xpercent.get() * gasPedal.get(), ypercent.get() * gasPedal.get(), tpercent.get());
    drivetrain.drive(
        xpercent.get() * gasPedal.get(), ypercent.get() * gasPedal.get(), tpercent.get(), true);
  }

  @Override
  public void end(boolean interupted) {}
}
