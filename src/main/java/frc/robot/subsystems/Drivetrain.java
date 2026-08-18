package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Drivetrain extends LightSubsystem implements Loggable {
  private SwerveDrive drive;
  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private SmartMotorControllerConfig buildDriveCfg() {
    return new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(0.03, 0, 0)
        .withGearing(new MechanismGearing(4.71))
        // .withFeedforward(
        //     new SimpleMotorFeedforward(
        //         0,
        //         12.0 / (MetersPerSecond.of(1).in(MetersPerSecond) / Inches.of(4).in(Meters)),
        //         0.01))
        .withStatorCurrentLimit(Amps.of(40));
    // .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
  }

  private SmartMotorControllerConfig buildAzimuthCfg() {
    var cfg =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(1, 0, 0)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // .withGearing(new GearBox(new double[]{12.8}))
            // .withFeedforward(new SimpleMotorFeedforward(0, 0))
            .withGearing(46.42)
            // .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
            .withStatorCurrentLimit(Amps.of(20));
    cfg.getPID(ClosedLoopControllerSlot.SLOT_0).ifPresent(pid -> pid.enableContinuousInput(0, 360));
    return cfg;
    // .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
  }

  public SwerveModule createModule(
      SparkMax drive,
      SparkMax azimuth,
      String moduleName,
      Translation2d location) { // , Angle angleZeroOffset) {
    SmartMotorController driveSMC = new SparkWrapper(drive, DCMotor.getNEO(1), buildDriveCfg());
    SmartMotorController azimuthSMC =
        new SparkWrapper(
            azimuth,
            DCMotor.getNeo550(1),
            buildAzimuthCfg()); // .withExternalEncoder(azimuth.getAbsoluteEncoder()));

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
            .withMaximumChassisSpeed(MetersPerSecond.of(4.8), RPM.of(80))
            // gyro.getYaw() gives the heading used for field-relative driving.
            .withGyro(() -> Degrees.of(gyro.getYaw()).plus(Degrees.of(180)))
            .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
            // Translation and rotation PIDs are used by driveToPose(); kP=1 is a conservative
            // start.
            .withTranslationController(new PIDController(1, 0, 0))
            .withRotationController(new PIDController(1, 0, 0));

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
    loggerhead.addStructArrayLogger(
        "swervestate", logMode, () -> drive.getModuleStates(), SwerveModuleState.struct);
    parentTable.addStructLogger("robot", logMode, drive::getPose, Pose2d.struct);
  }
}
