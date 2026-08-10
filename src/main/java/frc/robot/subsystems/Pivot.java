package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.PivotConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Pivot extends LightSubsystem implements Loggable {
  private CANcoder pivotEncoder = new CANcoder(PivotConstants.pivotEncoderID);

  private final TalonFX rawMotor = new TalonFX(PivotConstants.pivotMotorID);
  private final SmartMotorController motor =
      new TalonFXWrapper(
          rawMotor,
          DCMotor.getFalcon500(1),
          PivotConstants.pivotMotorConfig.withExternalEncoder(pivotEncoder).withSubsystem(this));

  public Trigger isAtSetpoint =
      new Trigger(
          () -> {
            var setpoint = motor.getMechanismPositionSetpoint();
            if (setpoint.isPresent()) {
              return setpoint.get().isNear(motor.getMechanismPosition(), 0.1);
            }
            return false;
          });

  public void setAngle(Angle angle) {
    motor.setPosition(angle);
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
    parentTable
        .addDoubleLogger("pivotAngle", logMode, () -> motor.getMechanismPosition().in(Degrees))
        .addBooleanLogger("isatSetpoint", logMode, () -> isAtSetpoint.getAsBoolean());
  }

  // @Override
  // public void periodic() {
  //   //        SmartDashboard.putNumber("pivot pos", pivotMotor.getPosition().getValueAsDouble());
  //   SmartDashboard.putBoolean("at pivot setpoint", atSetpoint.getAsBoolean());
  // }

  // public double getPivotPos() {
  //   return pivotMotor.getPosition().getValueAsDouble();
  // }

  // public Command goToPos(double pos) {
  //   return this.runOnce(
  //           () -> {
  //             setpoint = pos;
  //             pivotMotor.setControl(new MotionMagicVoltage(pos));
  //           })
  //       .andThen(Commands.waitUntil(atSetpoint));
  // }

  // public Trigger atSetpoint() {
  //   return atSetpoint;
  // }
}
