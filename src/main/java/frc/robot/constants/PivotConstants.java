package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.ArmFeedforward;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class PivotConstants {
  public static final int pivotMotorID = 12;
  public static final int pivotEncoderID = 13;

  public static final SmartMotorControllerConfig pivotMotorConfig =
      new SmartMotorControllerConfig()
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(50, 0, 0)
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withTelemetry("PivotMotor", TelemetryVerbosity.LOW)
          .withGearing(61.5385)
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withExternalEncoderInverted(false)
          .withExternalEncoderGearing(1);
}
