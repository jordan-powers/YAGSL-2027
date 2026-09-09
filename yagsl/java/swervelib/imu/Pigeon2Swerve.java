package swervelib.imu;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * SwerveIMU interface for the {@link Pigeon2}
 */
public class Pigeon2Swerve extends SwerveIMU
{

  /**
   * Wait time for status frames to show up.
   */
  public static double              STATUS_TIMEOUT_SECONDS = 0.04;
  /**
   * {@link Pigeon2} IMU device.
   */
  private final Pigeon2             imu;
  /**
   * X Acceleration supplier
   */
  private final Supplier<StatusSignal<LinearAcceleration>> xAcc;
  /**
   * Y Accelleration supplier.
   */
  private final Supplier<StatusSignal<LinearAcceleration>> yAcc;
  /**
   * Z Acceleration supplier.
   */
  private final Supplier<StatusSignal<LinearAcceleration>> zAcc;
  /**
   * Offset for the {@link Pigeon2}.
   */
  private       Rotation3d          offset                 = new Rotation3d();
  /**
   * Inversion for the gyro
   */
  private       boolean             invertedIMU            = false;
  /**
   * {@link Pigeon2} configurator.
   */
  private       Pigeon2Configurator cfg;

  /**
   * Generate the SwerveIMU for {@link Pigeon2}.
   *
   * @param canid  CAN ID for the {@link Pigeon2}
   * @param canbus CAN Bus name the {@link Pigeon2} resides on.
   */
  public Pigeon2Swerve(int canid, int canbus)
  {
    imu = new Pigeon2(canid, CANBus.systemcore(canbus));
    this.cfg = imu.getConfigurator();
    xAcc = imu::getAccelerationX;
    yAcc = imu::getAccelerationY;
    zAcc = imu::getAccelerationZ;
    // Telemetry.log(imu);
  }

  /**
   * Generate the SwerveIMU for {@link Pigeon2}.
   *
   * @param canid CAN ID for the {@link Pigeon2}
   */
  public Pigeon2Swerve(int canid)
  {
    this(canid, 0);
  }

  @Override
  public void close() {
    imu.close();
  }


  /**
   * Reset {@link Pigeon2} to factory default.
   */
  @Override
  public void factoryDefault()
  {
    Pigeon2Configuration config = new Pigeon2Configuration();

    // Compass utilization causes readings to jump dramatically in some cases.
    cfg.apply(config.Pigeon2Features.withEnableCompass(false));
  }

  /**
   * Clear sticky faults on {@link Pigeon2}.
   */
  @Override
  public void clearStickyFaults()
  {
    imu.clearStickyFaults();
  }

  /**
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public void setOffset(Rotation3d offset)
  {
    this.offset = offset;
  }

  /**
   * Set the gyro to invert its default direction
   *
   * @param invertIMU invert gyro direction
   */
  public void setInverted(boolean invertIMU)
  {
    invertedIMU = invertIMU;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRawRotation3d()
  {
    Rotation3d reading = imu.getRotation3d();
    return invertedIMU ? reading.inverse() : reading;
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRotation3d()
  {
    return getRawRotation3d().rotateBy(offset.inverse());
  }


  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel()
  {
    return Optional.of(new Translation3d(xAcc.get().getValueAsDouble(),
                                         yAcc.get().getValueAsDouble(),
                                         zAcc.get().getValueAsDouble()));
  }

  @Override
  public AngularVelocity getYawAngularVelocity()
  {
    return imu.getAngularVelocityZWorld().refresh().getValue();
  }

  /**
   * Get the instantiated {@link Pigeon2} object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
