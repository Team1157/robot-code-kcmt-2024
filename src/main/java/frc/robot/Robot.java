package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

public class Robot extends TimedRobot {
  private ADXRS450_Gyro gyro;
  private DifferentialDrivePoseEstimator m_poseEstimator;

  // Motor ports and PDP setup
  private static final int LEFT_MOTOR_PORT = 0;
  private static final int LEFT_FOLLOWER_PORT = 1;
  private static final int RIGHT_MOTOR_PORT = 2;
  private static final int RIGHT_FOLLOWER_PORT = 3;
  PowerDistribution m_pdp = new PowerDistribution(0, ModuleType.kCTRE);
  private final PWMTalonSRX m_leftMotor = new PWMTalonSRX(LEFT_MOTOR_PORT);
  private final PWMTalonSRX m_rightMotor = new PWMTalonSRX(RIGHT_MOTOR_PORT);
  private final PWMTalonSRX m_leftFollower = new PWMTalonSRX(LEFT_FOLLOWER_PORT);
  private final PWMTalonSRX m_rightFollower = new PWMTalonSRX(RIGHT_FOLLOWER_PORT);

  // Differential drive and controller setup
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final XboxController m_driverController = new XboxController(0);

  // Sensors and NetworkTables
  private BuiltInAccelerometer m_accelerometer;
  private NetworkTableInstance m_ntInstance;
  private NetworkTable m_dashboardTable;
  private NetworkTableEntry m_accelXEntry, m_accelYEntry, m_accelZEntry;
  private NetworkTableEntry m_leftMotorOutputEntry, m_rightMotorOutputEntry;
  private NetworkTableEntry m_leftFollowerOutputEntry, m_rightFollowerOutputEntry, m_camera;
  private NetworkTableEntry m_timerEntry, m_fieldPositionEntry, m_drivemode, m_slowmode;
  private NetworkTableEntry m_visionTargetEntry; // Vision target entry
  private final Timer m_timer = new Timer();

  // Field2d visualization
  private final Field2d m_field = new Field2d();

  // Initialize odometry
  private final double kTrackWidthMeters = 0.69; // Example track width (meters)
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  private DifferentialDriveOdometry m_odometry;
  //private Rotation2d Rotation2d;
  
  // Constructor to register motor objects with the SendableRegistry
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void robotInit() {
    // Initialize and calibrate gyro
    gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    gyro.calibrate();
    
    // Initialize NetworkTables for Elastic dashboard communication
    initializeNetworkTables();
    
    // Add field visualization to Elastic
    SmartDashboard.putData("Field", m_field);

    // Initialize odometry
    
    //m_odometry = new DifferentialDriveOdometry(Rotation2d, gyro.getAngle(), kTrackWidthMeters);
  }

  // Configure motor inversion and followers
  private void configureMotors() {
    m_leftMotor.addFollower(m_leftFollower);
    m_rightMotor.addFollower(m_rightFollower);
    m_rightMotor.setInverted(true); // Invert right motors so it won't just go in circles
    m_accelerometer = new BuiltInAccelerometer();
  }

  // Initialize NetworkTables for communication with elastic dashboard
  private void initializeNetworkTables() {
    configureMotors();
    m_ntInstance = NetworkTableInstance.getDefault();
    m_dashboardTable = m_ntInstance.getTable("SmartDashboard");
    m_accelXEntry = m_dashboardTable.getEntry("AccelX");
    m_accelYEntry = m_dashboardTable.getEntry("AccelY");
    m_accelZEntry = m_dashboardTable.getEntry("AccelZ");
    m_leftMotorOutputEntry = m_dashboardTable.getEntry("LeftMotorOutput");
    m_rightMotorOutputEntry = m_dashboardTable.getEntry("RightMotorOutput");
    m_leftFollowerOutputEntry = m_dashboardTable.getEntry("LeftFollowerOutput");
    m_rightFollowerOutputEntry = m_dashboardTable.getEntry("RightFollowerOutput");
    m_timerEntry = m_dashboardTable.getEntry("MatchTime");
    m_fieldPositionEntry = m_dashboardTable.getEntry("FieldPosition");
    m_drivemode = m_dashboardTable.getEntry("DriveMode");
    m_slowmode = m_dashboardTable.getEntry("SlowMode");
    m_visionTargetEntry = m_dashboardTable.getEntry("VisionTarget"); // Vision target entry
  }

  @Override
  public void robotPeriodic() {
    // Update gyro data
    double angle = gyro.getAngle();
    double rate = gyro.getRate();
    
    // Update NetworkTables with sensor data
    updateNetworkTables();
  
    // Display gyro values on SmartDashboard
    SmartDashboard.putNumber("Gyro Angle", angle);
    SmartDashboard.putNumber("Gyro Rate", rate);
  }

  // Send sensor and motor data to NetworkTables
  private void updateNetworkTables() {
    //m_accelXEntry.setDouble(m_accelerometer.getX());
    //m_accelYEntry.setDouble(m_accelerometer.getY());
    //m_accelZEntry.setDouble(m_accelerometer.getZ());

    // Send power distribution data
    double voltage = m_pdp.getVoltage();
    double temperatureCelsius = m_pdp.getTemperature();
    SmartDashboard.putNumber("Temperature", temperatureCelsius);
    SmartDashboard.putNumber("Voltage", voltage);
    SmartDashboard.putNumber("Total Current", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("Total Power", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Total Energy", m_pdp.getTotalEnergy());

    // Send motor outputs to dashboard
    m_leftMotorOutputEntry.setDouble(m_leftMotor.get());
    m_rightMotorOutputEntry.setDouble(m_rightMotor.get());
    m_leftFollowerOutputEntry.setDouble(m_leftFollower.get());
    m_rightFollowerOutputEntry.setDouble(m_rightFollower.get());

    // Send timer and field position data
    m_timerEntry.setDouble(Timer.getMatchTime());
    m_fieldPositionEntry.setString(m_field.getRobotPose().toString());

    // Display current for each PDP channel
    for (int channel = 0; channel <= 15; channel++) {
      double current = m_pdp.getCurrent(channel);
      SmartDashboard.putNumber("Current Channel " + channel, current);
    }

    // Update vision data
    double visionTargetX = m_visionTargetEntry.getDouble(0.0);
    SmartDashboard.putNumber("Vision Target X", visionTargetX);
  }

  @Override
  public void teleopPeriodic() {
    // Read controller inputs for field-relative drive
    double forwardSpeed = -m_driverController.getLeftY();
    double strafeSpeed = m_driverController.getLeftX();
    double rotationSpeed = -m_driverController.getRawAxis(5);

    // Get the current gyro angle
    double gyroAngle = gyro.getAngle();

    // Calculate pose estimate
    //Pose2d currentPose = m_odometry.update(
    //  Rotation2d, gyro.getAngle(), gyroAngle
    //);

    // Update field visualization with robot pose
    //m_field.setRobotPose(currentPose);

    // Apply field-relative drive
    driveFieldRelative(forwardSpeed, strafeSpeed, rotationSpeed, gyroAngle);

    // Reset gyro and calibrate gyro based on button inputs
    if (m_driverController.getRawButton(8)) {
      gyro.reset();
    }
    if (m_driverController.getRawButton(2)) {
      gyro.calibrate();
    }
  }

  /**
   * Implements field-relative drive using gyro feedback.
   *  
   * @param forward The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param strafe The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
   * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
   * @param gyroAngle The current gyro angle in degrees.
   */
  private void driveFieldRelative(double forward, double strafe, double rotation, double gyroAngle) {
    // Convert gyro angle to radians
    if (false == (m_driverController.getRawButton(5)))  {
      double gyroRadians = Math.toRadians(gyroAngle);
      m_drivemode.setString("Field centric :)");
      
      // Apply field-relative transformations
      double temp = forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
      strafe = -forward * Math.sin(gyroRadians) + strafe * Math.cos(gyroRadians);
      forward = temp;

      // Adjust inputs for differential drive
      double leftMotorOutput = forward + rotation;
      double rightMotorOutput = forward - rotation;

      // Make bot fasttttt
      leftMotorOutput *= 2;
      rightMotorOutput *= 1.8;

      // Slow mode
      if (m_driverController.getRawButton(6)) {
        m_slowmode.setString("True");
        leftMotorOutput *= 0.5;
        rightMotorOutput *= 0.5;
      } else {
        m_slowmode.setString("False");
      }
      // Set motor outputs
      m_robotDrive.tankDrive(leftMotorOutput, rightMotorOutput);
    } else {
      // Read controller inputs and control robot with arcade drive 
      m_drivemode.setString("robo centric :)");

      double speed = 2 * -m_driverController.getLeftY();
      rotation = 2 * m_driverController.getRawAxis(5);
      m_robotDrive.arcadeDrive(speed, rotation);
    }
  }
}