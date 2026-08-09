package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
  public static final double maxSpeed = Units.feetToMeters(14);
  public static final double driveGearRatio = 4.71;

  public static final double autoCollectP = 0.5;
  public static final double autoCollectI = 0;
  public static final double autoCollectD = 0;
  public static final double autoCollectForwardSpeed = 1;
  public static final double autoCollectMaxSideSpeed = 1;

  public static final double autoShootP = 1;
  public static final double autoShootI = 0;
  public static final double autoShootD = 0;
}
