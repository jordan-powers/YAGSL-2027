package swervelib.parser.json;

import static swervelib.telemetry.SwerveDriveTelemetry.canIdWarning;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;

import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.SwerveIMU;
import swervelib.imu.SyscoreIMU;
import swervelib.motors.SwerveMotor;
import swervelib.motors.TalonFXSwerve;

/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson
{

  /**
   * The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int    id;
  /**
   * The systemcore CAN bus which the device resides on if using CAN.
   */
  public int canbus = 0;

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the current configuration.
   *
   * @param motor {@link SwerveMotor} of which attached encoders will be created from, only used when the type is
   *              "attached" or "canandencoder".
   * @return {@link SwerveAbsoluteEncoder} given.
   */
  public SwerveAbsoluteEncoder createEncoder(SwerveMotor motor)
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "none":
        return null;
      case "cancoder":
        return new CANCoderSwerve(id, syscoreBus(canbus));
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveIMU} from the given configuration.
   *
   * @return {@link SwerveIMU} given.
   */
  public SwerveIMU createIMU()
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "syscore":
        return new SyscoreIMU();
      case "pigeon2":
        return new Pigeon2Swerve(id, syscoreBus(canbus));
      default:
        throw new RuntimeException(type + " is not a recognized imu/gyroscope type.");
    }
  }

  /**
   * Create a {@link SwerveMotor} from the given configuration.
   *
   * @param isDriveMotor If the motor being generated is a drive motor.
   * @return {@link SwerveMotor} given.
   */
  public SwerveMotor createMotor(boolean isDriveMotor)
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "krakenx60":
        return new TalonFXSwerve(id, syscoreBus(canbus), isDriveMotor, DCMotor.getKrakenX60(1));
      default:
        throw new RuntimeException(type + " is not a recognized motor type.");
    }

  }

  private static CANPort syscoreBus(int bus) {
    return switch(bus) {
        case 0 -> CANPort.CAN_S0;
        case 1 -> CANPort.CAN_S1;
        case 2 -> CANPort.CAN_S2;
        case 3 -> CANPort.CAN_S3;
        case 4 -> CANPort.CAN_S4;
        default -> throw new IllegalArgumentException("Invalid syscore bus [" + bus + "]");
    };
  }
}
