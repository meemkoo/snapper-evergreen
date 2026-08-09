package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Flywheel extends LightSubsystem implements Loggable {
  private final TalonFX rawMotor = new TalonFX(ShooterConstants.flywheelMotorCanId);
  private final SmartMotorController motor =
      new TalonFXWrapper(
          rawMotor,
          DCMotor.getFalcon500(1),
          ShooterConstants.flywheelMotorConfig.withSubsystem(this));

  public void setDutyCycle(double dutyCycle) {
    motor.setDutyCycle(dutyCycle);
  }

  public void setOff() {
    motor.setDutyCycle(0);
  }

  @Override
  public void periodic() {
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motor.simIterate();
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable.addDoubleLogger(
        "flywheelSpeed", logMode, () -> motor.getMechanismVelocity().in(RPM));
  }
}
