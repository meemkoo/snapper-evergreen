package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class IntakeConstants {
  public static final int intakeMotorCanId = 14;

  public static final SmartMotorControllerConfig intakeMotorConfig =
      new SmartMotorControllerConfig()
          .withControlMode(ControlMode.OPEN_LOOP)
          .withTelemetry("IntakeMotor", TelemetryVerbosity.LOW)
          .withGearing(1)
          .withMotorInverted(false)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40));
}
