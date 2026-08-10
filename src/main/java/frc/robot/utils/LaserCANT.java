package frc.robot.utils;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;

public class LaserCANT extends LaserCan {
  public static enum LaserCanStatus {
    ValidMeasurement(0),
    NoiseIssue(1),
    WeakSignal(2),
    OutOfBounds(4),
    Wraparound(7);

    public final int value;

    private LaserCanStatus(int value) {
      this.value = value;
    }
  }

  public static class MeasurementStruct implements Struct<Measurement> {
    @Override
    public Class<Measurement> getTypeClass() {
      return Measurement.class;
    }

    @Override
    public String getTypeName() {
      return "Measurement";
    }

    @Override
    public int getSize() {
      return kSizeInt32
          + kSizeInt32
          + kSizeInt32
          + kSizeBool
          + kSizeInt32
          + kSizeInt32
          + kSizeInt32
          + kSizeInt32
          + kSizeInt32;
    }

    @Override
    public String getSchema() {
      return "int32 status;int32 distance_mm;int32 ambient;bool is_long;int32 budget_ms;int32 x;int32 y;int32 w;int32 h";
    }

    @Override
    public Measurement unpack(ByteBuffer bb) {
      int status = bb.getInt();
      int distance_mm = bb.getInt();
      int ambient = bb.getInt();
      boolean is_long = bb.get() == 1;
      int budget_ms = bb.getInt();
      int x = bb.getInt();
      int y = bb.getInt();
      int w = bb.getInt();
      int h = bb.getInt();

      var roi = new RegionOfInterest(x, y, w, h);
      return new Measurement(status, distance_mm, ambient, is_long, budget_ms, roi);
    }

    @Override
    public void pack(ByteBuffer bb, Measurement em) {
      bb.putInt(em.status);
      bb.putInt(em.distance_mm);
      bb.putInt(em.ambient);
      bb.put((byte) (em.is_long ? 1 : 0));
      bb.putInt(em.budget_ms);
      bb.putInt(em.roi.x);
      bb.putInt(em.roi.y);
      bb.putInt(em.roi.w);
      bb.putInt(em.roi.h);
    }

    @Override
    public boolean isImmutable() {
      return false;
    }

    public static final MeasurementStruct struct = new MeasurementStruct();
  }

  private static final List<String> statusOptions =
      Arrays.asList(LaserCanStatus.values()).stream().map(r -> r.toString()).toList();

  private static final List<String> rangingModeOptions =
      Arrays.asList(RangingMode.values()).stream().map(r -> r.toString()).toList();
  private static final List<String> timingBudgetOptions =
      Arrays.asList(TimingBudget.values()).stream().map(r -> r.toString()).toList();

  private SimDevice simDevice;

  private SimEnum simStatus;
  private SimInt simDistance_mm;
  private SimInt simAmbient;
  private SimBoolean simIs_long;
  private SimInt simBudget_ms;

  private SimEnum simRangingMode;

  private SimEnum simTimingBudget;

  private SimInt simRoiX;
  private SimInt simRoiY;
  private SimInt simRoiW;
  private SimInt simRoiH;

  public LaserCANT(int canId) {
    super(canId);

    simDevice = SimDevice.create("WheezerCAN ", canId);

    if (simDevice != null) {
      simStatus =
          simDevice.createEnum(
              "measure.status", Direction.kInput, statusOptions.toArray(String[]::new), 0);

      simDistance_mm = simDevice.createInt("measure.distance_mm", Direction.kInput, 0);
      simAmbient = simDevice.createInt("measure.ambient", Direction.kInput, 0);
      simIs_long = simDevice.createBoolean("measure.is_long", Direction.kInput, false);
      simBudget_ms = simDevice.createInt("measure.budget_ms", Direction.kInput, 0);

      simRangingMode =
          simDevice.createEnum(
              "rangingMode", Direction.kBidir, rangingModeOptions.toArray(String[]::new), 0);

      simTimingBudget =
          simDevice.createEnum(
              "timingBudget", Direction.kBidir, timingBudgetOptions.toArray(String[]::new), 0);

      simRoiX = simDevice.createInt("ROI.x", Direction.kBidir, 0);
      simRoiY = simDevice.createInt("ROI.y", Direction.kBidir, 0);
      simRoiW = simDevice.createInt("ROI.w", Direction.kBidir, 0);
      simRoiH = simDevice.createInt("ROI.h", Direction.kBidir, 0);
    }
  }

  @Override
  public Measurement getMeasurement() {
    if (simDevice == null) {
      return super.getMeasurement();
    } else {
      var roi = new RegionOfInterest(simRoiX.get(), simRoiY.get(), simRoiW.get(), simRoiH.get());
      var simM =
          new Measurement(
              LaserCanStatus.valueOf(statusOptions.get(simStatus.get())).value,
              simDistance_mm.get(),
              simAmbient.get(),
              simIs_long.get(),
              simBudget_ms.get(),
              roi);
      return simM;
    }
  }

  @Override
  public void setRangingMode(RangingMode mode) throws ConfigurationFailedException {
    if (simDevice == null) {
      super.setRangingMode(mode);
    } else {
      simRangingMode.set(rangingModeOptions.indexOf(mode.toString()));
    }
  }

  @Override
  public void setTimingBudget(TimingBudget budget) throws ConfigurationFailedException {
    if (simDevice == null) {
      super.setTimingBudget(budget);
    } else {
      simTimingBudget.set(timingBudgetOptions.indexOf(budget.toString()));
    }
  }

  @Override
  public void setRegionOfInterest(RegionOfInterest roi) throws ConfigurationFailedException {
    if (simDevice == null) {
      super.setRegionOfInterest(roi);
    } else {
      simRoiX.set(roi.x);
      simRoiY.set(roi.y);
      simRoiW.set(roi.w);
      simRoiH.set(roi.h);
    }
  }
}
