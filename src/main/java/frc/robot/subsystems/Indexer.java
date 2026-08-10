package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.LaserCANT;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class Indexer extends LightSubsystem implements Loggable {
  private final SparkMax rawMotor =
      new SparkMax(ShooterConstants.indexerMotorCanId, MotorType.kBrushless);
  private final SmartMotorController motor =
      new SparkWrapper(
          rawMotor, DCMotor.getNeo550(1), ShooterConstants.indexerMotorConfig.withSubsystem(this));

  private LaserCan noteSensor;
  public Trigger noteSensorActive;
  private Alert laserCanAltert = new Alert("laserCanAlert", AlertType.kError);

  public Indexer() {
    noteSensor = new LaserCANT(ShooterConstants.laserCanCanId);
    try {
      noteSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCanAltert.set(false);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
      laserCanAltert.set(true);
      throw new RuntimeException();
    }

    noteSensorActive =
        new Trigger(
            () -> {
              int measure = noteSensor.getMeasurement().distance_mm;
              if ((Integer) measure != null) {
                return measure < 100;
              } else {
                return true;
              }
            });
  }

  public void setDutyCycle(double dutyCycle) {
    motor.setDutyCycle(dutyCycle);
  }

  public void setOff() {
    motor.setDutyCycle(0);
  }

  public void periodic() {
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motor.simIterate();
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable
        .addDoubleLogger("indexerSpeed", logMode, () -> motor.getMechanismVelocity().in(RPM))
        .addStructLogger(
            "noteSensor1",
            logMode,
            () -> noteSensor.getMeasurement(),
            LaserCANT.MeasurementStruct.struct);
  }
}
