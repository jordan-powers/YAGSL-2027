package swervelib;

import static org.wpilib.units.Units.Newtons;
import static org.wpilib.units.Units.RadiansPerSecond;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.Trajectory;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Notifier;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Force;
import org.wpilib.units.measure.LinearVelocity;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.parser.Cache;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.telemetry.Alert;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Swerve Drive class representing and controlling the swerve drive.
 */
public class SwerveDrive implements AutoCloseable
{

  /**
   * Swerve Kinematics object.
   */
  public final  SwerveDriveKinematics    kinematics;
  /**
   * Swerve drive configuration.
   */
  public final  SwerveDriveConfiguration swerveDriveConfiguration;
  /**
   * Swerve odometry.
   */
  public final  SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  /**
   * IMU reading cache for robot readings.
   */
  public final  Cache<Rotation3d>        imuReadingCache;
  /**
   * Swerve modules.
   */
  private final SwerveModule[]           swerveModules;
  /**
   * WPILib {@link Notifier} to keep odometry up to date.
   */
  private final Notifier                 odometryThread;
  /**
   * Odometry lock to ensure thread safety.
   */
  private final Lock                odometryLock                                    = new ReentrantLock();
  /**
   * Alert to recommend Tuner X if the configuration is compatible.
   */
  private final Alert               tunerXRecommendation                            = new Alert("Swerve Drive",
                                                                                                "Your Swerve Drive is compatible with Tuner X swerve generator, please consider using that instead of YAGSL. More information here!\n" +
                                                                                                "https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tuner-swerve/index.html",
                                                                                                Alert.AlertType.kWarning);
  /**
   * NT4 Publisher for the IMU reading.
   */
  private final DoublePublisher     rawIMUPublisher
                                                                                    = NetworkTableInstance.getDefault()
                                                                                                          .getTable(
                                                                                                              "SmartDashboard")
                                                                                                          .getDoubleTopic(
                                                                                                              "swerve/imu/raw")
                                                                                                          .publish();
  /**
   * NT4 Publisher for the IMU reading adjusted by offset and inversion.
   */
  private final DoublePublisher     adjustedIMUPublisher
                                                                                    = NetworkTableInstance.getDefault()
                                                                                                          .getTable(
                                                                                                              "SmartDashboard")
                                                                                                          .getDoubleTopic(
                                                                                                              "swerve/imu/adjusted")
                                                                                                          .publish();
  /**
   * Field object.
   */
  public        Field2d             field                                           = new Field2d();
  /**
   * Swerve controller for controlling heading of the robot.
   */
  public        SwerveController         swerveController;
  /**
   * Correct chassis velocity in {@link SwerveDrive#drive(Translation2d, double, boolean, boolean)} using 254's
   * correction.
   */
  public        boolean             chassisVelocityCorrection                       = true;
  /**
   * Correct chassis velocity in {@link SwerveDrive#setChassisVelocities(ChassisVelocities ChassisVelocities)} (auto) using 254's
   * correction during auto.
   */
  public        boolean             autonomousChassisVelocityCorrection             = false;
  /**
   * Correct for skew that scales with angular velocity in
   * {@link SwerveDrive#drive(Translation2d, double, boolean, boolean)}
   */
  public        boolean             angularVelocityCorrection                       = false;
  /**
   * Correct for skew that scales with angular velocity in
   * {@link SwerveDrive#setChassisVelocities(ChassisVelocities ChassisVelocities)} during auto.
   */
  public        boolean             autonomousAngularVelocityCorrection             = false;
  /**
   * Angular Velocity Correction Coefficent (expected values between -0.15 and 0.15).
   */
  public        double              angularVelocityCoefficient                      = 0;
  /**
   * Whether to correct heading when driving translationally. Set to true to enable.
   */
  public        boolean             headingCorrection                               = false;
  /**
   * Amount of seconds the duration of the timestep the speeds should be applied for.
   */
  private       double              discretizationdtSeconds                         = 0.02;
  /**
   * Deadband for speeds in heading correction.
   */
  private       double              HEADING_CORRECTION_DEADBAND                     = 0.01;
  /**
   * Swerve IMU device for sensing the heading of the robot.
   */
  private       SwerveIMU                imu;
  /**
   * Counter to synchronize the modules relative encoder with absolute encoder when not moving.
   */
  private       int                 moduleSynchronizationCounter                    = 0;
  /**
   * The last heading set in radians.
   */
  private       double              lastHeadingRadians                              = 0;
  /**
   * The absolute max speed that your robot can reach while translating in meters per second.
   */
  private       double              attainableMaxTranslationalSpeedMetersPerSecond  = 0;
  /**
   * The absolute max speed the robot can reach while rotating radians per second.
   */
  private       double              attainableMaxRotationalVelocityRadiansPerSecond = 0;
  /**
   * Maximum speed of the robot in meters per second.
   */
  private       double              maxChassisSpeedMPS;

  /**
   * Creates a new swerve drivebase subsystem. Robot is controlled via the {@link SwerveDrive#drive} method, or via the
   * {@link SwerveDrive#setRawModuleStates} method. The {@link SwerveDrive#drive} method incorporates kinematics-- it
   * takes a translation and rotation, as well as parameters for field-centric and closed-loop velocity control.
   * {@link SwerveDrive#setRawModuleStates} takes a list of swerveModuleVelocities and directly passes them to the modules.
   * This subsystem also handles odometry.
   *
   * @param config           The {@link SwerveDriveConfiguration} configuration to base the swerve drive off of.
   * @param controllerConfig The {@link SwerveControllerConfiguration} to use when creating the
   *                         {@link SwerveController}.
   * @param maxSpeedMPS      Maximum speed of the robot in meters per second, remember to use
   *                         {@link Units#feetToMeters(double)} if you have feet per second!
   * @param startingPose     Starting {@link Pose2d} on the field.
   */
  public SwerveDrive(
      SwerveDriveConfiguration config, SwerveControllerConfiguration controllerConfig, double maxSpeedMPS,
      Pose2d startingPose)
  {
    this.attainableMaxTranslationalSpeedMetersPerSecond = this.maxChassisSpeedMPS = maxSpeedMPS;
    this.attainableMaxRotationalVelocityRadiansPerSecond = Math.PI *
                                                           2; // Defaulting to something reasonable for most robots
    swerveDriveConfiguration = config;
    swerveController = new SwerveController(controllerConfig);
    // Create Kinematics from swerve module locations.
    kinematics = new SwerveDriveKinematics(config.moduleLocationsMeters);
    odometryThread = new Notifier(this::updateOdometry);

    this.swerveModules = config.modules;

    imu = config.imu;
    imu.factoryDefault();
    imuReadingCache = new Cache<>(imu::getRotation3d, 5L);

    //    odometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());
    swerveDrivePoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getYaw(),
            getModulePositions(),
            startingPose); // x,y,heading in radians; Vision measurement std dev, higher=less weight
//
//    Rotation3d currentGyro = imuReadingCache.getValue();
//    double offset = currentGyro.getZ() +
//                    startingPose.getRotation().getRadians();
//    setGyroOffset(new Rotation3d(currentGyro.getX(), currentGyro.getY(), offset));

    // Initialize Telemetry
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.POSE.ordinal())
    {
      SmartDashboard.putData("Field", field);
    }

    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.INFO.ordinal())
    {
      SwerveDriveTelemetry.maxSpeed = maxSpeedMPS;
      SwerveDriveTelemetry.maxAngularVelocity = swerveController.config.maxAngularVelocity;
      SwerveDriveTelemetry.moduleCount = swerveModules.length;
      SwerveDriveTelemetry.sizeFrontBack = Units.metersToInches(SwerveMath.getSwerveModule(swerveModules, true,
                                                                                           false).moduleLocation.getX() +
                                                                SwerveMath.getSwerveModule(swerveModules,
                                                                                           false,
                                                                                           false).moduleLocation.getX());
      SwerveDriveTelemetry.sizeLeftRight = Units.metersToInches(SwerveMath.getSwerveModule(swerveModules, false,
                                                                                           true).moduleLocation.getY() +
                                                                SwerveMath.getSwerveModule(swerveModules,
                                                                                           false,
                                                                                           false).moduleLocation.getY());
      SwerveDriveTelemetry.wheelLocations = new double[SwerveDriveTelemetry.moduleCount * 2];
      for (SwerveModule module : swerveModules)
      {
        SwerveDriveTelemetry.wheelLocations[module.moduleNumber * 2] = Units.metersToInches(
            module.configuration.moduleLocation.getX());
        SwerveDriveTelemetry.wheelLocations[(module.moduleNumber * 2) + 1] = Units.metersToInches(
            module.configuration.moduleLocation.getY());
      }
      SwerveDriveTelemetry.measuredStates = new double[SwerveDriveTelemetry.moduleCount * 2];
      SwerveDriveTelemetry.desiredStates = new double[SwerveDriveTelemetry.moduleCount * 2];
      SwerveDriveTelemetry.desiredStatesObj = new SwerveModuleVelocity[SwerveDriveTelemetry.moduleCount];
      SwerveDriveTelemetry.measuredStatesObj = new SwerveModuleVelocity[SwerveDriveTelemetry.moduleCount];
    }

    setOdometryPeriod( 0.02);

    checkIfTunerXCompatible();
  }

  @Override
  public void close()
  {
    imu.close();
    tunerXRecommendation.close();

    for (var module : swerveModules)
    {
      module.close();
    }
  }

  /**
   * Update the cache validity period for the robot.
   *
   * @param imu             IMU reading cache validity period in milliseconds.
   * @param driveMotor      Drive motor reading cache in milliseconds.
   * @param absoluteEncoder Absolute encoder reading cache in milliseconds.
   */
  public void updateCacheValidityPeriods(long imu, long driveMotor, long absoluteEncoder)
  {
    imuReadingCache.updateValidityPeriod(imu);
    for (SwerveModule module : swerveModules)
    {
      module.drivePositionCache.updateValidityPeriod(driveMotor);
      module.driveVelocityCache.updateValidityPeriod(driveMotor);
      module.absolutePositionCache.updateValidityPeriod(absoluteEncoder);
    }
  }

  /**
   * Check all components to ensure that Tuner X Swerve Generator is recommended instead.
   */
  private void checkIfTunerXCompatible()
  {
//    boolean compatible = imu instanceof Pigeon2Swerve;
//    for (SwerveModule module : swerveModules)
//    {
//      compatible = compatible && module.getDriveMotor() instanceof TalonFXSwerve &&
//                   module.getAngleMotor() instanceof TalonFXSwerve &&
//                   module.getAbsoluteEncoder() instanceof CANCoderSwerve;
//      if (!compatible)
//      {
//        break;
//      }
//    }
//    if (compatible)
//    {
//      tunerXRecommendation.set(true);
//    }

  }

  /**
   * Set the odometry update period in seconds.
   *
   * @param period period in seconds.
   */
  public void setOdometryPeriod(double period)
  {
    odometryThread.stop();
    odometryThread.startPeriodic(period);
  }

  /**
   * Stop the odometry thread in favor of manually updating odometry.
   */
  public void stopOdometryThread()
  {
    odometryThread.stop();
  }

  /**
   * Set the conversion factor for the angle/azimuth motor controller.
   *
   * @param conversionFactor Angle motor conversion factor for PID, should be generated from
   *                         {@link SwerveMath#calculateDegreesPerSteeringRotation(double, double)} or calculated.
   */
  public void setAngleMotorConversionFactor(double conversionFactor)
  {
    for (SwerveModule module : swerveModules)
    {
      module.setAngleMotorConversionFactor(conversionFactor);
    }
  }

  /**
   * Set the conversion factor for the drive motor controller.
   *
   * @param conversionFactor Drive motor conversion factor for PID, should be generated from
   *                         {@link SwerveMath#calculateMetersPerRotation(double, double, double)} or calculated.
   */
  public void setDriveMotorConversionFactor(double conversionFactor)
  {
    for (SwerveModule module : swerveModules)
    {
      module.setDriveMotorConversionFactor(conversionFactor);
    }
  }

  /**
   * Fetch the latest odometry heading, should be trusted over {@link SwerveDrive#getYaw()}.
   *
   * @return {@link Rotation2d} of the robot heading.
   */
  public Rotation2d getOdometryHeading()
  {
    return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Set the heading correction capabilities of YAGSL.
   *
   * @param state {@link SwerveDrive#headingCorrection} state.
   */
  public void setHeadingCorrection(boolean state)
  {
    setHeadingCorrection(state, HEADING_CORRECTION_DEADBAND);
  }

  /**
   * Set the heading correction capabilities of YAGSL.
   *
   * @param state    {@link SwerveDrive#headingCorrection} state.
   * @param deadband {@link SwerveDrive#HEADING_CORRECTION_DEADBAND} deadband.
   */
  public void setHeadingCorrection(boolean state, double deadband)
  {
    headingCorrection = state;
    HEADING_CORRECTION_DEADBAND = deadband;
  }

  /**
   * Tertiary method of controlling the drive base given velocity in both field oriented and robot oriented at the same
   * time. The inputs are added together so this is not intended to be used to give the driver both methods of control.
   *
   * @param fieldOrientedVelocity The field oriented velocties to use
   * @param robotOrientedVelocity The robot oriented velocties to use
   */
  public void driveFieldOrientedAndRobotOriented(ChassisVelocities fieldOrientedVelocity,
                                                 ChassisVelocities robotOrientedVelocity)
  {

    drive(fieldOrientedVelocity.toRobotRelative(getOdometryHeading())
                       .plus(robotOrientedVelocity));
  }

  /**
   * Secondary method of controlling the drive base given velocity and adjusting it for field oriented use.
   *
   * @param fieldRelativeSpeeds Velocity of the robot desired.
   */
  public void driveFieldOriented(ChassisVelocities fieldRelativeSpeeds)
  {
    drive(fieldRelativeSpeeds.toRobotRelative(getOdometryHeading()));
  }

  /**
   * Secondary method of controlling the drive base given velocity and adjusting it for field oriented use.
   *
   * @param fieldRelativeSpeeds    Velocity of the robot desired.
   * @param centerOfRotationMeters The center of rotation in meters, 0 is the center of the robot.
   */
  public void driveFieldOriented(ChassisVelocities fieldRelativeSpeeds, Translation2d centerOfRotationMeters)
  {
    drive(fieldRelativeSpeeds.toRobotRelative(getOdometryHeading()), centerOfRotationMeters);
  }

  /**
   * Secondary method for controlling the drivebase. Given a simple {@link ChassisVelocities} set the swerve module states,
   * to achieve the goal.
   *
   * @param velocity The desired robot-oriented {@link ChassisVelocities} for the robot to achieve.
   */
  public void drive(ChassisVelocities velocity)
  {
    drive(velocity, false, new Translation2d());
  }

  /**
   * Secondary method for controlling the drivebase. Given a simple {@link ChassisVelocities} set the swerve module states,
   * to achieve the goal.
   *
   * @param velocity               The desired robot-oriented {@link ChassisVelocities} for the robot to achieve.
   * @param centerOfRotationMeters The center of rotation in meters, 0 is the center of the robot.
   */
  public void drive(ChassisVelocities velocity, Translation2d centerOfRotationMeters)
  {
    drive(velocity, false, centerOfRotationMeters);
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate, and calculates
   * and commands module states accordingly. Can use either open-loop or closed-loop velocity control for the wheel
   * velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation            {@link Translation2d} that is the commanded linear velocity of the robot, in meters
   *                               per second. In robot-relative mode, positive x is torwards the bow (front) and
   *                               positive y is torwards port (left). In field-relative mode, positive x is away from
   *                               the alliance wall (field North) and positive y is torwards the left wall when looking
   *                               through the driver station glass (field West).
   * @param rotation               Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
   *                               relativity.
   * @param fieldRelative          Drive mode. True for field-relative, false for robot-relative.
   * @param isOpenLoop             Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   * @param centerOfRotationMeters The center of rotation in meters, 0 is the center of the robot.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
      Translation2d centerOfRotationMeters)
  {
    // Creates a robot-relative ChassisVelocities object, converting from field-relative speeds if
    // necessary.
    ChassisVelocities velocity = new ChassisVelocities(translation.getX(), translation.getY(), rotation);
    if (fieldRelative)
    {
      velocity = velocity.toRobotRelative(getOdometryHeading());
    }
    drive(velocity, isOpenLoop, centerOfRotationMeters);
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate, and calculates
   * and commands module states accordingly. Can use either open-loop or closed-loop velocity control for the wheel
   * velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x is away from the alliance wall (field
   *                      North) and positive y is torwards the left wall when looking through the driver station glass
   *                      (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    // Creates a robot-relative ChassisVelocities object, converting from field-relative speeds if
    // necessary.
    ChassisVelocities velocity = new ChassisVelocities(translation.getX(), translation.getY(), rotation);

    if (fieldRelative)
    {
      velocity = velocity.toRobotRelative(getOdometryHeading());
    }
    drive(velocity, isOpenLoop, new Translation2d());
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link ChassisVelocities}, and calculates and commands module
   * states accordingly. Can use either open-loop or closed-loop velocity control for the wheel velocities. Applies
   * heading correction if enabled and necessary.
   *
   * @param robotRelativeVelocity  The chassis speeds to set the robot to achieve.
   * @param isOpenLoop             Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   * @param centerOfRotationMeters The center of rotation in meters, 0 is the center of the robot.
   */
  public void drive(ChassisVelocities robotRelativeVelocity, boolean isOpenLoop, Translation2d centerOfRotationMeters)
  {
    SwerveDriveTelemetry.startCtrlCycle();
    robotRelativeVelocity = movementOptimizations(robotRelativeVelocity,
                                                  chassisVelocityCorrection,
                                                  angularVelocityCorrection);

    // Heading Angular Velocity Deadband, might make a configuration option later.
    // Originally made by Team 1466 Webb Robotics.
    // Modified by Team 7525 Pioneers and BoiledBurntBagel of 6036
    if (headingCorrection)
    {
      if (Math.abs(robotRelativeVelocity.omega) < HEADING_CORRECTION_DEADBAND
          && (Math.abs(robotRelativeVelocity.vx) > HEADING_CORRECTION_DEADBAND
              || Math.abs(robotRelativeVelocity.vy) > HEADING_CORRECTION_DEADBAND))
      {
        robotRelativeVelocity.omega =
            swerveController.headingCalculate(getOdometryHeading().getRadians(), lastHeadingRadians);
      } else
      {
        lastHeadingRadians = getOdometryHeading().getRadians();
      }
    }

    // Display commanded speed for testing
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.LOW.ordinal())
    {
      SwerveDriveTelemetry.desiredChassisVelocitiesObj = robotRelativeVelocity;
    }

    // Calculate required module states via kinematics
    SwerveModuleVelocity[] swerveModuleVelocities = kinematics.toSwerveModuleVelocities(robotRelativeVelocity,
                                                                             centerOfRotationMeters);

    setRawModuleStates(swerveModuleVelocities, robotRelativeVelocity, isOpenLoop);
  }

  /**
   * Set the maximum attainable speeds for desaturation.
   *
   * @param attainableMaxTranslationalSpeedMetersPerSecond  The absolute max speed that your robot can reach while
   *                                                        translating in meters per second.
   * @param attainableMaxRotationalVelocityRadiansPerSecond The absolute max speed the robot can reach while rotating in
   *                                                        radians per second.
   */
  public void setMaximumAttainableSpeeds(
      double attainableMaxTranslationalSpeedMetersPerSecond,
      double attainableMaxRotationalVelocityRadiansPerSecond)
  {
    this.attainableMaxTranslationalSpeedMetersPerSecond = attainableMaxTranslationalSpeedMetersPerSecond;
    this.attainableMaxRotationalVelocityRadiansPerSecond = attainableMaxRotationalVelocityRadiansPerSecond;
  }

  /**
   * Set the maximum allowable speeds for desaturation.
   *
   * @param maxTranslationalSpeedMetersPerSecond  The allowable max speed that your robot should reach while translating
   *                                              in meters per second.
   * @param maxRotationalVelocityRadiansPerSecond The allowable max speed the robot should reach while rotating in
   *                                              radians per second.
   */
  public void setMaximumAllowableSpeeds(
      double maxTranslationalSpeedMetersPerSecond,
      double maxRotationalVelocityRadiansPerSecond)
  {
    this.maxChassisSpeedMPS = maxTranslationalSpeedMetersPerSecond;
    this.swerveController.config.maxAngularVelocity = maxRotationalVelocityRadiansPerSecond;
  }

  /**
   * Get the maximum velocity from {@link SwerveDrive#attainableMaxTranslationalSpeedMetersPerSecond} or
   * {@link SwerveDrive#maxChassisSpeedMPS} whichever is the lower limit on the robot's speed.
   *
   * @return Minimum speed in meters/second of physically attainable and user allowable limits.
   */
  public double getMaximumChassisVelocity()
  {
    return Math.min(this.attainableMaxTranslationalSpeedMetersPerSecond, maxChassisSpeedMPS);
  }

  /**
   * Get the maximum drive velocity of a module as a {@link LinearVelocity}.
   *
   * @return {@link LinearVelocity} representing the maximum drive speed of a module.
   */
  public double getMaximumModuleDriveVelocity()
  {
    return swerveModules[0].getMaxDriveVelocityMetersPerSecond();
  }

  /**
   * Get the maximum angular velocity of an azimuth/angle motor in the swerve module.
   *
   * @return {@link AngularVelocity} of the maximum azimuth/angle motor.
   */
  public AngularVelocity getMaximumModuleAngleVelocity()
  {
    return swerveModules[0].getMaxAngularVelocity();
  }

  /**
   * Get the maximum angular velocity, either {@link SwerveDrive#attainableMaxRotationalVelocityRadiansPerSecond} or
   * {@link SwerveControllerConfiguration#maxAngularVelocity}, whichever is the lower limit on the robot's speed.
   *
   * @return Minimum angular velocity in radians per second of physically attainable and user allowable limits.
   */
  public double getMaximumChassisAngularVelocity()
  {
    return Math.min(this.attainableMaxRotationalVelocityRadiansPerSecond, swerveController.config.maxAngularVelocity);
  }

  /**
   * Set the module states (azimuth and velocity) directly.
   *
   * @param desiredStates       A list of swerveModuleVelocities to send to the modules.
   * @param desiredChassisSpeed The desired chassis speeds to set the robot to achieve.
   * @param isOpenLoop          Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  private void setRawModuleStates(SwerveModuleVelocity[] desiredStates, ChassisVelocities desiredChassisSpeed,
                                  boolean isOpenLoop)
  {
    // Desaturates wheel speeds
    double maxModuleSpeedMPS = getMaximumModuleDriveVelocity();
    if ((attainableMaxTranslationalSpeedMetersPerSecond != 0 || attainableMaxRotationalVelocityRadiansPerSecond != 0) &&
        attainableMaxTranslationalSpeedMetersPerSecond != maxChassisSpeedMPS)
    {
      SwerveDriveKinematics.desaturateWheelVelocities(desiredStates, desiredChassisSpeed,
                                                  maxModuleSpeedMPS,
                                                  attainableMaxTranslationalSpeedMetersPerSecond,
                                                  attainableMaxRotationalVelocityRadiansPerSecond);
    } else
    {
      SwerveDriveKinematics.desaturateWheelVelocities(desiredStates, maxModuleSpeedMPS);
    }

    // Sets states
    for (SwerveModule module : swerveModules)
    {
      module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop, false);
    }
  }

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto paths. Does not allow for usage of
   * {@link SwerveDriveKinematics#desaturateWheelVelocities(SwerveModuleVelocity[] moduleStates, ChassisVelocities
   * desiredChassisSpeed, double attainableMaxModuleSpeedMetersPerSecond, double
   * attainableMaxTranslationalSpeedMetersPerSecond, double attainableMaxRotationalVelocityRadiansPerSecond)}
   *
   * @param desiredStates A list of swerveModuleVelocities to send to the modules.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true to disable closed-loop.
   */
  public void setModuleStates(SwerveModuleVelocity[] desiredStates, boolean isOpenLoop)
  {
    SwerveDriveTelemetry.startCtrlCycle();
    double maxModuleSpeedMPS = getMaximumModuleDriveVelocity();
    desiredStates = kinematics.toSwerveModuleVelocities(kinematics.toChassisVelocities(desiredStates));
    SwerveDriveKinematics.desaturateWheelVelocities(desiredStates, maxModuleSpeedMPS);

    // Sets states
    for (SwerveModule module : swerveModules)
    {
      module.setDesiredState(desiredStates[module.moduleNumber], isOpenLoop, false);
    }
  }

  /**
   * Drive the robot using the {@link SwerveModuleVelocity}, it is recommended to have
   * {@link SwerveDrive#setCosineCompensator(boolean)} set to false for this.<br/>
   *
   * @param robotRelativeVelocity Robot relative {@link ChassisVelocities}
   * @param states                Corresponding {@link SwerveModuleVelocity} to use (not checked against the
   *                              {@param robotRelativeVelocity}).
   * @param feedforwardForces     Feedforward forces generated by set-point generator
   */
  public void drive(ChassisVelocities robotRelativeVelocity, SwerveModuleVelocity[] states, Force[] feedforwardForces)
  {
    SwerveDriveTelemetry.startCtrlCycle();
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.LOW.ordinal())
    {
      SwerveDriveTelemetry.desiredChassisVelocitiesObj = robotRelativeVelocity;
    }
    for (SwerveModule module : swerveModules)
    {
      module.applyStateOptimizations(states[module.moduleNumber]);
      module.applyAntiJitter(states[module.moduleNumber], false);

      // from the module configuration, obtain necessary information to calculate feed-forward
      // Warning: Will not work well if motor is not what we are expecting.
      // Warning: Should replace module.getDriveMotor().simMotor with expected motor type first.
      DCMotor driveMotorModel   = module.configuration.driveMotor.getSimMotor();
      double  driveGearRatio    = module.configuration.conversionFactors.drive.gearRatio;
      double  wheelRadiusMeters = Units.inchesToMeters(module.configuration.conversionFactors.drive.diameter) / 2;

      // calculation:
      double desiredGroundSpeedMPS = states[module.moduleNumber].velocity;
      double feedforwardVoltage = driveMotorModel.getVoltage(
          // Since: (1) torque = force * momentOfForce; (2) torque (on wheel) = torque (on motor) * gearRatio
          // torque (on motor) = force * wheelRadius / gearRatio
          feedforwardForces[module.moduleNumber].in(Newtons) * wheelRadiusMeters / driveGearRatio,
          // Since: (1) linear velocity = angularVelocity * wheelRadius; (2) wheelVelocity = motorVelocity / gearRatio
          // motorAngularVelocity = linearVelocity / wheelRadius * gearRatio
          desiredGroundSpeedMPS / wheelRadiusMeters * driveGearRatio
                                                            );
      module.setDesiredState(
          states[module.moduleNumber],
          false,
          feedforwardVoltage
                            );
    }
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param robotRelativeSpeeds Chassis speeds to set.
   */
  public void setChassisVelocities(ChassisVelocities robotRelativeSpeeds)
  {
    SwerveDriveTelemetry.startCtrlCycle();
    robotRelativeSpeeds = movementOptimizations(robotRelativeSpeeds,
                                                autonomousChassisVelocityCorrection,
                                                autonomousAngularVelocityCorrection);

    SwerveDriveTelemetry.desiredChassisVelocitiesObj = robotRelativeSpeeds;

    setRawModuleStates(kinematics.toSwerveModuleVelocities(robotRelativeSpeeds), robotRelativeSpeeds, false);
  }

  /**
   * Gets the measured pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {

    odometryLock.lock();
    Pose2d poseEstimation = swerveDrivePoseEstimator.getEstimatedPosition();
    odometryLock.unlock();
    return poseEstimation;
  }

  /**
   * Gets the measured field-relative robot velocity (x, y and omega)
   *
   * @return A ChassisVelocities object of the current field-relative velocity
   */
  public ChassisVelocities getFieldVelocity()
  {
    // ChassisVelocities has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the
    // angle given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    ChassisVelocities robotRelativeSpeeds = kinematics.toChassisVelocities(getStates());
    return robotRelativeSpeeds.toFieldRelative(getOdometryHeading());
    // Might need to be this instead
    //return ChassisVelocities.fromFieldRelativeSpeeds(
    //        kinematics.toChassisVelocities(getStates()), getOdometryHeading().unaryMinus());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisVelocities object of the current robot-relative velocity
   */
  public ChassisVelocities getRobotVelocity()
  {
    return kinematics.toChassisVelocities(getStates());
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param pose The pose to set the odometry to. Field relative, blue-origin where 0deg is facing towards RED alliance.
   */
  public void resetOdometry(Pose2d pose)
  {
    odometryLock.lock();
    swerveDrivePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    odometryLock.unlock();
    ChassisVelocities robotRelativeSpeeds = new ChassisVelocities(0, 0, 0).toFieldRelative(getYaw());
    kinematics.toSwerveModuleVelocities(robotRelativeSpeeds);

  }

  /**
   * Post the trajectory to the field
   *
   * @param trajectory the trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.POSE.ordinal())
    {
      field.getObject("Trajectory").setTrajectory(trajectory);
    }
  }

  /**
   * Gets the current module states (azimuth and velocity)
   *
   * @return A list of swerveModuleVelocities containing the current module states
   */
  public SwerveModuleVelocity[] getStates()
  {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[swerveDriveConfiguration.moduleCount];
    for (SwerveModule module : swerveModules)
    {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters)).
   *
   * @return A list of SwerveModulePositions containg the current module positions
   */
  public SwerveModulePosition[] getModulePositions()
  {
    SwerveModulePosition[] positions =
        new SwerveModulePosition[swerveDriveConfiguration.moduleCount];
    for (SwerveModule module : swerveModules)
    {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  /**
   * Getter for the {@link SwerveIMU}.
   *
   * @return generated {@link SwerveIMU}
   */
  public SwerveIMU getGyro()
  {
    return swerveDriveConfiguration.imu;
  }

  /**
   * Set the expected gyroscope angle using a {@link Rotation3d} object. To reset gyro, set to a new {@link Rotation3d}
   * subtracted from the current gyroscopic readings {@link SwerveIMU#getRotation3d()}.
   *
   * @param gyro expected gyroscope angle as {@link Rotation3d}.
   */
  public void setGyro(Rotation3d gyro)
  {

    setGyroOffset(imu.getRawRotation3d().rotateBy(gyro.inverse()));
    imuReadingCache.update();
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0 (red alliance station).
   */
  public void zeroGyro()
  {
    // Resets the real gyro or the angle accumulator, depending on whether the robot is being
    // simulated
    setGyroOffset(imu.getRawRotation3d());
    imuReadingCache.update();
    swerveController.lastAngleScalar = 0;
    lastHeadingRadians = 0;
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw as a {@link Rotation2d} angle
   */
  public Rotation2d getYaw()
  {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    return Rotation2d.fromRadians(imuReadingCache.getValue().getZ());
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    return Rotation2d.fromRadians(imuReadingCache.getValue().getY());
  }

  /**
   * Gets the current roll angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getRoll()
  {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    return Rotation2d.fromRadians(imuReadingCache.getValue().getX());
  }

  /**
   * Gets the current gyro {@link Rotation3d} of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation3d} angle
   */
  public Rotation3d getGyroRotation3d()
  {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    return imuReadingCache.getValue();
  }

  /**
   * Gets current acceleration of the robot in m/s/s. If gyro unsupported returns empty.
   *
   * @return acceleration of the robot as a {@link Translation3d}
   */
  public Optional<Translation3d> getAccel()
  {
    return imu.getAccel();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorIdleMode(boolean brake)
  {
    for (SwerveModule swerveModule : swerveModules)
    {
      swerveModule.setMotorBrake(brake);
    }
  }

  /**
   * Enable auto synchronization for encoders during a match. This will only occur when the modules are not moving for a
   * few seconds.
   *
   * @param enabled  Enable state
   * @param deadband Deadband in degrees, default is 3 degrees.
   */
  public void setModuleEncoderAutoSynchronize(boolean enabled, double deadband)
  {
    for (SwerveModule swerveModule : swerveModules)
    {
      swerveModule.setEncoderAutoSynchronize(enabled, deadband);
    }
  }


  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep
   * the current pose.
   */
  public void lockPose()
  {
    // Sets states
    for (SwerveModule swerveModule : swerveModules)
    {
      SwerveModuleVelocity desiredState =
          new SwerveModuleVelocity(0, swerveModule.configuration.moduleLocation.getAngle());
      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.INFO.ordinal())
      {
        SwerveDriveTelemetry.desiredStatesObj[swerveModule.moduleNumber] = desiredState;
      }
      swerveModule.setDesiredState(desiredState, false, true);

    }

    // Update kinematics because we are not using setModuleStates
    kinematics.toSwerveModuleVelocities(new ChassisVelocities());
  }

  /**
   * Get the swerve module poses and on the field relative to the robot.
   *
   * @param robotPose Robot pose.
   * @return Swerve module poses.
   */
  public Pose2d[] getSwerveModulePoses(Pose2d robotPose)
  {
    Pose2d[]     poseArr = new Pose2d[swerveDriveConfiguration.moduleCount];
    List<Pose2d> poses   = new ArrayList<>();
    for (SwerveModule module : swerveModules)
    {
      poses.add(
          robotPose.plus(
              new Transform2d(module.configuration.moduleLocation, module.getState().angle)));
    }
    return poses.toArray(poseArr);
  }

  /**
   * Setup the swerve module feedforward.
   *
   * @param driveFeedforward Feedforward for the drive motor on swerve modules.
   */
  public void replaceSwerveModuleFeedforward(SimpleMotorFeedforward driveFeedforward)
  {
    for (SwerveModule swerveModule : swerveModules)
    {
      swerveModule.setFeedforward(driveFeedforward);
    }
  }

  /**
   * Update odometry should be run every loop. Synchronizes module absolute encoders with relative encoders
   * periodically. In simulation mode will also post the pose of each module. Updates SmartDashboard with module encoder
   * readings and states.
   */
  public void updateOdometry()
  {
    SwerveDriveTelemetry.startOdomCycle();
    odometryLock.lock();
//    invalidateCache();
    try
    {
      // Update odometry
      swerveDrivePoseEstimator.update(getYaw(), getModulePositions());

      // Update angle accumulator if the robot is simulated
      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.INFO.ordinal())
      {
        SwerveDriveTelemetry.measuredChassisVelocitiesObj = getRobotVelocity();
        SwerveDriveTelemetry.robotRotationObj = getOdometryHeading();
      }

      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.POSE.ordinal())
      {
        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
      }

      double sumVelocity = 0;
      for (SwerveModule module : swerveModules)
      {
        SwerveModuleVelocity moduleState = module.getState();
        sumVelocity += Math.abs(moduleState.velocity);
        if (SwerveDriveTelemetry.verbosity == TelemetryVerbosity.HIGH)
        {
          module.updateTelemetry();
          rawIMUPublisher.set(getYaw().getDegrees());
          adjustedIMUPublisher.set(getOdometryHeading().getDegrees());
        }
        if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.INFO.ordinal())
        {
          SwerveDriveTelemetry.measuredStatesObj[module.moduleNumber] = moduleState;
        }
      }

      // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's SDS
      // lib)
      // To ensure that everytime we initialize it works.
      if (sumVelocity <= .01 && ++moduleSynchronizationCounter > 5)
      {
        synchronizeModuleEncoders();
        moduleSynchronizationCounter = 0;
      }

      if (SwerveDriveTelemetry.verbosity.ordinal() >= TelemetryVerbosity.INFO.ordinal())
      {
        SwerveDriveTelemetry.updateData();
      }
    } catch (Exception e)
    {
      odometryLock.unlock();
      throw e;
    }
    odometryLock.unlock();
    SwerveDriveTelemetry.endOdomCycle();
  }

  /**
   * Invalidate all {@link Cache} object used by the {@link SwerveDrive}
   */
  public void invalidateCache()
  {
    imuReadingCache.update();
    for (SwerveModule module : swerveModules)
    {
      module.invalidateCache();
    }
  }

  /**
   * Synchronize angle motor integrated encoders with data from absolute encoders.
   */
  public void synchronizeModuleEncoders()
  {
    for (SwerveModule module : swerveModules)
    {
      module.queueSynchronizeEncoders();
    }
  }

  /**
   * Set the gyro scope offset to a desired known rotation. Unlike {@link SwerveDrive#setGyro(Rotation3d)} it DOES NOT
   * take the current rotation into account.
   *
   * @param offset {@link Rotation3d} known offset of the robot for gyroscope to use.
   */
  public void setGyroOffset(Rotation3d offset)
  {
    imu.setOffset(offset);
    imuReadingCache.update();
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement.
   *
   * @param robotPose                Robot {@link Pose2d} as measured by vision.
   * @param timestamp                Timestamp the measurement was taken as time since startup, should be taken from
   *                                 {@link Timer#getMonotonicTimestamp()} or similar sources.
   * @param visionMeasurementStdDevs Vision measurement standard deviation that will be sent to the
   *                                 {@link SwerveDrivePoseEstimator}.The standard deviation of the vision measurement,
   *                                 for best accuracy calculate the standard deviation at 2 or more points and fit a
   *                                 line to it with the calculated optimal standard deviation. (Units should be meters
   *                                 per pixel). By optimizing this you can get * vision accurate to inches instead of
   *                                 feet.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp,
                                   Matrix<N3, N1> visionMeasurementStdDevs)
  {
    odometryLock.lock();
    swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
    odometryLock.unlock();
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in vision measurements
   * after the autonomous period, or to change trust as distance to a vision target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these numbers to trust
   *                                 global measurements from vision less. This matrix is in the form [x, y, theta],
   *                                 with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs)
  {
    odometryLock.lock();
    swerveDrivePoseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    odometryLock.unlock();
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the {@link SwerveIMU} gyro reading with
   * the given timestamp of the vision measurement.
   *
   * @param robotPose Robot {@link Pose2d} as measured by vision.
   * @param timestamp Timestamp the measurement was taken as time since startup, should be taken from
   *                  {@link Timer#getMonotonicTimestamp()} or similar sources.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp)
  {
    odometryLock.lock();
    swerveDrivePoseEstimator.addVisionMeasurement(robotPose, timestamp);
//    Pose2d newOdometry = new Pose2d(swerveDrivePoseEstimator.getEstimatedPosition().getTranslation(),
//                                    robotPose.getRotation());
    odometryLock.unlock();

//    setGyroOffset(new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
//    resetOdometry(newOdometry);
  }

  /**
   * Helper function to get the {@link SwerveDrive#swerveController} for the {@link SwerveDrive} which can be used to
   * generate {@link ChassisVelocities} for the robot to orient it correctly given axis or angles, and apply
   * {@link org.wpilib.math.filter.SlewRateLimiter} to given inputs. Important functions to look at are
   * {@link SwerveController#getTargetSpeeds(double, double, double, double, double)},
   * {@link SwerveController#addSlewRateLimiters(SlewRateLimiter, SlewRateLimiter, SlewRateLimiter)},
   * {@link SwerveController#getRawTargetSpeeds(double, double, double)}.
   *
   * @return {@link SwerveController} for the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveController;
  }

  /**
   * Get the {@link SwerveModule}s associated with the {@link SwerveDrive}.
   *
   * @return {@link SwerveModule} array specified by configurations.
   */
  public SwerveModule[] getModules()
  {
    return swerveDriveConfiguration.modules;
  }

  /**
   * Get the {@link SwerveModule}'s as a {@link HashMap} where the key is the swerve module configuration name.
   *
   * @return {@link HashMap}(Module Name, SwerveModule)
   */
  public Map<String, SwerveModule> getModuleMap()
  {
    Map<String, SwerveModule> map = new HashMap<String, SwerveModule>();
    for (SwerveModule module : swerveModules)
    {
      map.put(module.configuration.name, module);
    }
    return map;
  }

  /**
   * Reset the drive encoders on the robot, useful when manually resetting the robot without a reboot, like in
   * autonomous.
   */
  public void resetDriveEncoders()
  {
    for (SwerveModule module : swerveModules)
    {
      module.getDriveMotor().setPosition(0);
    }
  }

  /**
   * Set the motor controller closed loop feedback device to the defined external absolute encoder, with the given
   * offset from the supplied configuration, overwriting any native offset.
   */
  public void useExternalFeedbackSensor()
  {
    for (SwerveModule module : swerveModules)
    {
      module.useExternalFeedbackSensor();
    }
  }

  /**
   * Set the motor controller closed loop feedback device to the internal encoder instead of the absolute encoder.
   */
  public void useInternalFeedbackSensor()
  {
    for (SwerveModule module : swerveModules)
    {
      module.useInternalFeedbackSensor();
    }
  }

  /**
   * Pushes the Absolute Encoder offsets to the Encoder or Motor Controller, depending on type. Also removes the
   * internal offsets to prevent double offsetting.
   */
  @Deprecated
  public void pushOffsetsToEncoders()
  {
    for (SwerveModule module : swerveModules)
    {
      module.pushOffsetsToEncoders();
    }
  }

  /**
   * Restores Internal YAGSL Encoder offsets and sets the Encoder stored offset back to 0
   */
  @Deprecated
  public void restoreInternalOffset()
  {
    for (SwerveModule module : swerveModules)
    {
      module.restoreInternalOffset();
    }
  }

  /**
   * Set module optimization to be utilized or not. Sometimes it is desirable to be enabled for debugging purposes
   * only.
   *
   * @param enabled Optimization enabled state.
   */
  public void setModuleStateOptimization(boolean enabled)
  {
    for (SwerveModule module : swerveModules)
    {
      module.setModuleStateOptimization(enabled);
    }
  }

  /**
   * Enable auto-centering module wheels. This has a side effect of causing some jitter to the robot when a PID is not
   * tuned perfectly. This function is a wrapper for {@link SwerveModule#setAntiJitter(boolean)} to perform
   * auto-centering.
   *
   * @param enabled Enable auto-centering (disable antiJitter)
   */
  public void setAutoCenteringModules(boolean enabled)
  {
    for (SwerveModule module : swerveModules)
    {
      module.setAntiJitter(!enabled);
    }
  }

  /**
   * Enable or disable the {@link swervelib.parser.SwerveModuleConfiguration#useCosineCompensator} for all
   * {@link SwerveModule}'s in the swerve drive. The cosine compensator will slow down or speed up modules that are
   * close to their desired state in theory.
   *
   * @param enabled Usage of the cosine compensator.
   */
  public void setCosineCompensator(boolean enabled)
  {
    for (SwerveModule module : swerveModules)
    {
      module.configuration.useCosineCompensator = enabled;
    }
  }

  /**
   * Sets the Chassis discretization seconds as well as enableing/disabling the Chassis velocity correction in teleop
   *
   * @param enable    Enable chassis velocity correction, which will use
   *                  {@link ChassisVelocities#discretize(ChassisVelocities, double)}} with the following.
   * @param dtSeconds The duration of the timestep the speeds should be applied for.
   */
  public void setChassisDiscretization(boolean enable, double dtSeconds)
  {
    chassisVelocityCorrection = enable;
    discretizationdtSeconds = dtSeconds;
  }

  /**
   * Sets the Chassis discretization seconds as well as enableing/disabling the Chassis velocity correction in teleop
   * and/or auto
   *
   * @param useInTeleop Enable chassis velocity correction, which will use
   *                    {@link ChassisVelocities#discretize(ChassisVelocities, double)} with the following in teleop.
   * @param useInAuto   Enable chassis velocity correction, which will use
   *                    {@link ChassisVelocities#discretize(ChassisVelocities, double)} with the following in auto.
   * @param dtSeconds   The duration of the timestep the speeds should be applied for.
   */
  public void setChassisDiscretization(boolean useInTeleop, boolean useInAuto, double dtSeconds)
  {
    chassisVelocityCorrection = useInTeleop;
    autonomousChassisVelocityCorrection = useInAuto;
    discretizationdtSeconds = dtSeconds;
  }

  /**
   * Enables angular velocity skew correction in teleop and/or autonomous and sets the angular velocity coefficient for
   * both modes
   *
   * @param useInTeleop          Enables angular velocity correction in teleop.
   * @param useInAuto            Enables angular velocity correction in autonomous.
   * @param angularVelocityCoeff The angular velocity coefficient. Expected values between -0.15 to 0.15. Start with a
   *                             value of 0.1, test in teleop. When enabling for the first time if the skew is
   *                             significantly worse try inverting the value. Tune by moving in a straight line while
   *                             rotating. Testing is best done with angular velocity controls on the right stick.
   *                             Change the value until you are visually happy with the skew. Ensure your tune works
   *                             with different translational and rotational magnitudes. If this reduces skew in teleop,
   *                             it may improve auto.
   */
  public void setAngularVelocityCompensation(boolean useInTeleop, boolean useInAuto, double angularVelocityCoeff)
  {
    angularVelocityCorrection = useInTeleop;
    autonomousAngularVelocityCorrection = useInAuto;
    angularVelocityCoefficient = angularVelocityCoeff;
  }

  /**
   * Correct for skew that worsens as angular velocity increases
   *
   * @param robotRelativeVelocity The chassis speeds to set the robot to achieve.
   * @return {@link ChassisVelocities} of the robot after angular velocity skew correction.
   */
  public ChassisVelocities angularVelocitySkewCorrection(ChassisVelocities robotRelativeVelocity)
  {
    var angularVelocity = new Rotation2d(imu.getYawAngularVelocity().in(RadiansPerSecond) * angularVelocityCoefficient);
    if (angularVelocity.getRadians() != 0.0)
    {
      ChassisVelocities fieldRelativeVelocity = robotRelativeVelocity.toFieldRelative(
                                                                                  getOdometryHeading());
      robotRelativeVelocity = fieldRelativeVelocity.toRobotRelative(
                                                                    getOdometryHeading().plus(angularVelocity));
    }
    return robotRelativeVelocity;
  }

  /**
   * Enable desired drive corrections
   *
   * @param robotRelativeVelocity            The chassis speeds to set the robot to achieve.
   * @param uesChassisDiscretize             Correct chassis velocity using 254's correction.
   * @param useAngularVelocitySkewCorrection Use the robot's angular velocity to correct for skew.
   * @return The chassis speeds after optimizations.
   */
  private ChassisVelocities movementOptimizations(ChassisVelocities robotRelativeVelocity, boolean uesChassisDiscretize,
                                              boolean useAngularVelocitySkewCorrection)
  {

    if (useAngularVelocitySkewCorrection)
    {
      robotRelativeVelocity = angularVelocitySkewCorrection(robotRelativeVelocity);
    }

    // Thank you to Jared Russell FRC254 for Open Loop Compensation Code
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
    if (uesChassisDiscretize)
    {
      robotRelativeVelocity = robotRelativeVelocity.discretize(discretizationdtSeconds);
    }

    return robotRelativeVelocity;
  }

  /**
   * Convert a {@link ChassisVelocities} to {@link SwerveModuleVelocity[]} for use elsewhere.
   *
   * @param robotRelativeVelocity {@link ChassisVelocities} velocity to use.
   * @param optimize              Perform chassis velocity correction or angular velocity correction.
   * @return {@link SwerveModuleVelocity[]} for use elsewhere.
   */
  public SwerveModuleVelocity[] toServeModuleStates(ChassisVelocities robotRelativeVelocity, boolean optimize)
  {
    if (optimize)
    {
      robotRelativeVelocity = movementOptimizations(robotRelativeVelocity,
                                                    chassisVelocityCorrection,
                                                    angularVelocityCorrection);
    }
    return kinematics.toSwerveModuleVelocities(robotRelativeVelocity);
  }
}
