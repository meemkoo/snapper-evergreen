package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ShooterConstants {
  public static final int flywheelMotorCanId = 9;
  public static final int indexerMotorCanId = 10;
  public static final int laserCanCanId = 11;

  public static final SmartMotorControllerConfig flywheelMotorConfig =
      new SmartMotorControllerConfig()
          .withControlMode(ControlMode.OPEN_LOOP)
          .withTelemetry("FlywheelMotor", TelemetryVerbosity.LOW)
          .withGearing(1)
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40));

  public static final SmartMotorControllerConfig indexerMotorConfig =
      new SmartMotorControllerConfig()
          .withControlMode(ControlMode.OPEN_LOOP)
          .withTelemetry("IndexerMotor", TelemetryVerbosity.LOW)
          .withGearing(1)
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(30));
}
