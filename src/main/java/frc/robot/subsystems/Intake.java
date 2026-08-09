package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.IntakeConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class Intake extends LightSubsystem implements Loggable {
  private final SparkMax rawMotor =
      new SparkMax(IntakeConstants.intakeMotorCanId, MotorType.kBrushless);
  private final SmartMotorController motor =
      new SparkWrapper(
          rawMotor, DCMotor.getNEO(1), IntakeConstants.intakeMotorConfig.withSubsystem(this));

  public void setDutyCycle(double dutyCycle) {
    motor.setDutyCycle(dutyCycle);
  }

  public void setOn() {
    motor.setDutyCycle(1);
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
    parentTable.addDoubleLogger("intakeSpeed", logMode, () -> motor.getMechanismVelocity().in(RPM));
  }
}
