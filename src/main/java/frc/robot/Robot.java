package frc.robot;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private ADXRS450_Gyro gyro;

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
  private NetworkTableEntry m_leftFollowerOutputEntry, m_rightFollowerOutputEntry;
  private NetworkTableEntry m_timerEntry, m_fieldPositionEntry;

  // Autonomous mode options
  private static final String kDefaultAuto = "auto0";
  private static final String kCustomAuto1 = "auto1";
  private static final String kCustomAuto2 = "auto2";
  private static final String kCustomAuto3 = "auto3";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Timer m_timer = new Timer();

  // Field2d visualization
  private final Field2d m_field = new Field2d();

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

    //sets heading to 0
    gyro.reset();
    // Configure autonomous choices and motors
    configureAutoChooser();
    configureMotors();
    
    // Initialize NetworkTables for Elastic dashboard communication
    initializeNetworkTables();
    
    // Add field visualization to Elastic
    SmartDashboard.putData("Field", m_field);
  }

  // Configure autonomous mode chooser
  private void configureAutoChooser() {
    m_chooser.setDefaultOption("Forward for 4 seconds", kDefaultAuto);
    m_chooser.addOption("Forward, turn right, stop", kCustomAuto1);
    m_chooser.addOption("Forward, turn left, stop", kCustomAuto2);
    m_chooser.addOption("Forward, turn right, move forward again, stop", kCustomAuto3);
    SmartDashboard.putData("Auto choices", m_chooser);
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
    m_accelXEntry.setDouble(m_accelerometer.getX());
    m_accelYEntry.setDouble(m_accelerometer.getY());
    m_accelZEntry.setDouble(m_accelerometer.getZ());

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
  }

  @Override
  public void teleopPeriodic() {
      // Read controller inputs
      double leftX = 1.5 * m_driverController.getLeftX(); // Left stick X axis (for field-oriented direction)
      double leftY = 1.5 * m_driverController.getLeftY(); // Left stick Y axis (for forward/backward)
      double manualRotation = m_driverController.getRawAxis(5); // Right stick X axis (manual turning)
  
      // Check if the A button is pressed to reset gyro, idk how its mapped to the gcn controller
      if (m_driverController.getAButtonPressed()) {
          gyro.reset();
          SmartDashboard.putString("Gyro Reset", "Gyro reset at time: " + Timer.getFPGATimestamp());
      }
  
      // Calculate desired direction angle from the left stick
      double desiredAngle = Math.toDegrees(Math.atan2(leftX, -leftY)); // Negate leftY because forward is negative
      if (desiredAngle < 0) {
          desiredAngle += 360; // Ensure the angle is positive
      }
  
      // Get the current robot heading from the gyro
      double currentAngle = gyro.getAngle() % 360; // Ensure angle is between 0 and 360
      if (currentAngle < 0) {
          currentAngle += 360;
      }
  
      // Calculate the rotational difference (error) between desired and current angle
      double angleError = desiredAngle - currentAngle;
  
      // Normalize the angle error between -180 and 180 degrees
      if (angleError > 180) {
          angleError -= 360;
      } else if (angleError < -180) {
          angleError += 360;
      }
  
      // If the driver is manually turning, use that instead of auto-aligning the angle
      double rotationSpeed;
      if (Math.abs(manualRotation) > 0.1) {
          rotationSpeed = manualRotation * 0.5; // Manual turning (adjust sensitivity as needed)
      } else {
          // Auto-align based on the angle error (proportional control)
          rotationSpeed = angleError * 0.05; // 0.05 is the tuning factor for rotation correction
      }
  
      // Calculate speed based on the left stick Y axis (forward/backward)
      double speed = -leftY * 0.6; // Forward and backward movement, scale speed as needed
  
      // Drive the robot using arcade drive with field-oriented rotation control
      m_robotDrive.arcadeDrive(speed, rotationSpeed);
  }
  
  @Override
  public void autonomousInit() {
    // Get the selected autonomous routine
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    // Run the selected autonomous routine
    switch (m_autoSelected) {
      case kCustomAuto1:
        runAutoRoutine1();
        break;
      case kCustomAuto2:
        runAutoRoutine2();
        break;
      case kCustomAuto3:
        runAutoRoutine3();
        break;
      case kDefaultAuto:
      default:
        runDefaultAuto();
        break;
    }

    // Update field visualization
    updateField2d();
  }

  // Auto routine 1: Forward, turn right, stop
  private void runAutoRoutine1() {
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // Move forward at 50% speed
    } else if (m_timer.get() < 3.0) {
      m_robotDrive.arcadeDrive(0.0, 0.5); // Turn right at 50% speed
    } else {
      m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
    }
  }

  // Auto routine 2: Forward, turn left, stop
  private void runAutoRoutine2() {
    if (m_timer.get() < 3.0) {
      m_robotDrive.arcadeDrive(0.6, 0.0); // Move forward at 60% speed
    } else if (m_timer.get() < 4.5) {
      m_robotDrive.arcadeDrive(0.0, -0.6); // Turn left at 60% speed
    } else {
      m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
    }
  }

  // Auto routine 3: Forward, turn right, move forward again, stop
  private void runAutoRoutine3() {
    if (m_timer.get() < 1.5) {
      m_robotDrive.arcadeDrive(0.7, 0.0); // Move forward at 70% speed
    } else if (m_timer.get() < 3.5) {
      m_robotDrive.arcadeDrive(0.0, 0.7); // Turn right at 70% speed
    } else if (m_timer.get() < 5.0) {
      m_robotDrive.arcadeDrive(0.7, 0.0); // Move forward again at 70% speed
    } else {
      m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
    }
  }

  // Default auto routine: Move forward, stop
  private void runDefaultAuto() {
    if (m_timer.get() < 4.0) {
      m_robotDrive.arcadeDrive(1.0, 0.0); // Move forward at 100% speed
    } else {
      m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
    }
  }

  // Update field visualization with current robot pose that doesn't actually work lol
  private void updateField2d() {
    m_field.setRobotPose(m_field.getRobotPose());
  }
}
