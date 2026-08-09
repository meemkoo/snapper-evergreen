package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomPivotAngleSource {
  public CustomPivotAngleSource() {
    SmartDashboard.putNumber("customPivotAngleDegrees", 0);
  }

  public Angle getAngle() {
    return Degrees.of(SmartDashboard.getNumber("customPivotAngleDegrees", 0));
  }
}
