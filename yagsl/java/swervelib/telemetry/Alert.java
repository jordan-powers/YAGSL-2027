package swervelib.telemetry;

import org.wpilib.driverstation.DriverStationErrors;

/**
 * Thread-safe psuedo-Alert class
 */
public class Alert
{
    public enum AlertType {
        kInfo,
        kWarning,
        kError;
    }


  /**
   * Group of the alert
   */
  public final String    group;
  /**
   * Text of the alert
   */
  private String text;
  /**
   * Type of the alert
   */
  public final Alert.AlertType type;
  private      boolean   toggle = false;

  /**
   * Create a new Alert
   *
   * @param group Group of the alert
   * @param text  Text of the alert
   * @param type  Type of the alert
   */
  public Alert(String group, String text, Alert.AlertType type)
  {
    this.group = group;
    this.text = text;
    this.type = type;
  }

  /**
   * Create a new Alert
   *
   * @param text Text of the alert
   * @param type Type of the alert
   */
  public Alert(String text, Alert.AlertType type)
  {
    this("", text, type);
  }


  /**
   * Toggle the alert
   *
   * @param toggle
   */
  public void set(boolean toggle)
  {
    this.toggle = toggle;
    if (toggle)
    {
      String msg = "[" + group + "] " + text;
      switch (type)
      {
        case kError:
          DriverStationErrors.reportError(msg, toggle);
          break;
        case kInfo:
        case kWarning:
          DriverStationErrors.reportWarning(msg, toggle);
          break;
      }
    }
  }

  /***
   * Set the text of the alert
   * @param text Text of the alert
   */
  public void setText(String text)
  {
    this.text = text;
  }

  /// Does nothing
  public void close()
  {
    set(false);
  }
}
