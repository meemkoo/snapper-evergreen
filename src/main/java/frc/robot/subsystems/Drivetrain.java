package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Drivetrain extends LightSubsystem implements Loggable {
  private SwerveDrive drive;
  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private SmartMotorControllerConfig buildDriveCfg(String name) {
    return new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(0.3, 0, 0)
        .withGearing(new MechanismGearing(4.71))
        .withFeedforward(
            new SimpleMotorFeedforward(
                0,
                12.0 / (MetersPerSecond.of(1).in(MetersPerSecond) / Inches.of(4).in(Meters)),
                0.01))
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor" + name, TelemetryVerbosity.HIGH);
  }

  private SmartMotorControllerConfig buildAzimuthCfg(String name) {
    var config =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(1, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, 1))
            .withGearing(new MechanismGearing(46.423645320197))
            .withStatorCurrentLimit(Amps.of(20))
            .withContinuousWrapping(Degrees.of(0), Degrees.of(360))
            .withTelemetry("angleMotor" + name, TelemetryVerbosity.HIGH);

    return config;
  }

  public SwerveModule createModule(
      SparkMax drive, SparkMax azimuth, String moduleName, Translation2d location) {
    SmartMotorController driveSMC =
        new SparkWrapper(drive, DCMotor.getNEO(1), buildDriveCfg(moduleName));
    SmartMotorController azimuthSMC =
        new SparkWrapper(
            azimuth,
            DCMotor.getNeo550(1),
            buildAzimuthCfg(moduleName).withExternalEncoder(azimuth.getAbsoluteEncoder()));

    return new SwerveModule(
        new SwerveModuleConfig(driveSMC, azimuthSMC)
            .withAbsoluteEncoder(
                () -> azimuthSMC.getExternalEncoderPosition().orElseGet(() -> Degrees.of(67)))
            .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
            .withLocation(location)
            // State optimization rotates the module at most 90 deg instead of 180 deg + reversing
            // drive.
            .withOptimization(true));
  }

  public Drivetrain() {
    // Module locations: +X forward, +Y left. 24-inch offsets assume module
    // centers are 24 in from the robot center — update to match your chassis.
    // CAN IDs are grouped as (drive, steer, CANcoder) per module.
    var fl =
        createModule(
            new SparkMax(1, MotorType.kBrushless),
            new SparkMax(2, MotorType.kBrushless),
            "frontleft",
            new Translation2d(Inches.of(11.25984), Inches.of(11.25984)));
    var fr =
        createModule(
            new SparkMax(3, MotorType.kBrushless),
            new SparkMax(4, MotorType.kBrushless),
            "frontright",
            new Translation2d(Inches.of(11.25984), Inches.of(-11.25984)));
    var bl =
        createModule(
            new SparkMax(5, MotorType.kBrushless),
            new SparkMax(6, MotorType.kBrushless),
            "backleft",
            new Translation2d(Inches.of(-11.25984), Inches.of(11.25984)));
    var br =
        createModule(
            new SparkMax(7, MotorType.kBrushless),
            new SparkMax(8, MotorType.kBrushless),
            "backright",
            new Translation2d(Inches.of(-11.25984), Inches.of(-11.25984)));

    SwerveDriveConfig config =
        new SwerveDriveConfig(this, fl, fr, bl, br)
            .withDataLogName("Swerve")
            .withTelemetry(TelemetryVerbosity.HIGH)
            .withMaximumChassisSpeed(MetersPerSecond.of(4.8), RPM.of(80))
            .withGyro(() -> Degrees.of(gyro.getAngle()))
            .withStartingPose(new Pose2d(4, 4, Rotation2d.fromDegrees(0)));

    drive = new SwerveDrive(config);
  }

  public void drive(ChassisSpeeds fieldRelativeChassisSpeeds) {
    drive.setFieldRelativeChassisSpeeds(fieldRelativeChassisSpeeds);
  }

  @Override
  public void periodic() {
    drive.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable.addStructLogger("robot", logMode, drive::getPose, Pose2d.struct);
  }
}
