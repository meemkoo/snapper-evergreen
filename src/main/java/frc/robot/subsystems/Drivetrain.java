package frc.robot.subsystems;

import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.Loggable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import java.io.File;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drivetrain extends LightSubsystem implements Loggable {
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive drive;

  public Drivetrain() {
    try {
      drive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(
                  DrivetrainConstants.maxSpeed,
                  360,
                  SwerveMath.calculateMetersPerRotation(
                      Units.inchesToMeters(3), DrivetrainConstants.driveGearRatio, 1));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    drive.setCosineCompensator(false);
    drive.pushOffsetsToEncoders();
    drive.useExternalFeedbackSensor();
    drive.setHeadingCorrection(false);
  }

  @Override
  public void periodic() {
    drive.updateOdometry();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    drive.drive(
        translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  private double filter(double input) {
    return Math.pow(MathUtil.applyDeadband(input, 0.03), 3);
  }

  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier heading,
      BooleanSupplier isFieldOriented) {
    return this.run(
        () ->
            drive(
                new Translation2d(
                    -filter(translationX.getAsDouble()) * drive.getMaximumChassisVelocity(),
                    -filter(translationY.getAsDouble()) * drive.getMaximumChassisVelocity()),
                -filter(heading.getAsDouble()) * drive.getMaximumChassisAngularVelocity(),
                isFieldOriented.getAsBoolean()));
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    drive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void lock() {
    drive.lockPose();
  }
}
