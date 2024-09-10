package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final int LEFT_MOTOR_PORT = 0;
  private static final int LEFT_FOLLOWER_PORT = 1;
  private static final int RIGHT_MOTOR_PORT = 2;
  private static final int RIGHT_FOLLOWER_PORT = 3;

  private final PWMTalonSRX m_leftMotor = new PWMTalonSRX(LEFT_MOTOR_PORT);
  private final PWMTalonSRX m_rightMotor = new PWMTalonSRX(RIGHT_MOTOR_PORT);
  private final PWMTalonSRX m_leftFollower = new PWMTalonSRX(LEFT_FOLLOWER_PORT);
  private final PWMTalonSRX m_rightFollower = new PWMTalonSRX(RIGHT_FOLLOWER_PORT);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final XboxController m_driverController = new XboxController(0);

  private BuiltInAccelerometer m_accelerometer;
  private NetworkTableInstance m_ntInstance;
  private NetworkTable m_dashboardTable;
  private NetworkTableEntry m_accelXEntry, m_accelYEntry, m_accelZEntry;
  private NetworkTableEntry m_leftMotorOutputEntry, m_rightMotorOutputEntry;
  private NetworkTableEntry m_leftFollowerOutputEntry, m_rightFollowerOutputEntry;
  private NetworkTableEntry m_timerEntry, m_fieldPositionEntry;

  private static final String kDefaultAuto = "auto0";
  private static final String kCustomAuto1 = "auto1";
  private static final String kCustomAuto2 = "auto2";
  private static final String kCustomAuto3 = "auto3";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Timer m_timer = new Timer();

  private final Field2d m_field = new Field2d();

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void robotInit() {
    configureAutoChooser();
    configureMotors();
    initializeNetworkTables();
    SmartDashboard.putData("Field", m_field);
  }

  private void configureAutoChooser() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Custom Auto 1", kCustomAuto1);
    m_chooser.addOption("Custom Auto 2", kCustomAuto2);
    m_chooser.addOption("Custom Auto 3", kCustomAuto3);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  private void configureMotors() {
    m_leftMotor.addFollower(m_leftFollower);
    m_rightMotor.addFollower(m_rightFollower);
    m_rightMotor.setInverted(true);
    m_accelerometer = new BuiltInAccelerometer();
  }

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
    updateNetworkTables();
  }

  private void updateNetworkTables() {
    m_accelXEntry.setDouble(m_accelerometer.getX());
    m_accelYEntry.setDouble(m_accelerometer.getY());
    m_accelZEntry.setDouble(m_accelerometer.getZ());

    m_leftMotorOutputEntry.setDouble(m_leftMotor.get());
    m_rightMotorOutputEntry.setDouble(m_rightMotor.get());
    m_leftFollowerOutputEntry.setDouble(m_leftFollower.get());
    m_rightFollowerOutputEntry.setDouble(m_rightFollower.get());

    m_timerEntry.setDouble(Timer.getMatchTime());

    m_fieldPositionEntry.setString(m_field.getRobotPose().toString());
  }

  @Override
  public void teleopPeriodic() {
    double speed = -m_driverController.getLeftY();
    double rotation = -m_driverController.getRightX();
    m_robotDrive.arcadeDrive(speed, rotation);
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
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
    updateField2d();
  }

  private void runAutoRoutine1() {
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // Move forward with 50% speed
    } else if (m_timer.get() < 3.0) {
      m_robotDrive.arcadeDrive(0.0, 0.5); // Turn right with 50% speed
    } else {
      m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
    }
  }

  private void runAutoRoutine2() {
    if (m_timer.get() < 3.0) {
      m_robotDrive.arcadeDrive(0.6, 0.0); // Move forward with 60% speed
    } else if (m_timer.get() < 4.5) {
      m_robotDrive.arcadeDrive(0.0, -0.6); // Turn left with 60% speed
    } else {
      m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
    }
  }

  private void runAutoRoutine3() {
    if (m_timer.get() < 1.5) {
      m_robotDrive.arcadeDrive(0.7, 0.0); // Move forward with 70% speed
    } else if (m_timer.get() < 3.5) {
      m_robotDrive.arcadeDrive(0.0, 0.7); // Turn right with 70% speed
    } else if (m_timer.get() < 5.0) {
      m_robotDrive.arcadeDrive(0.7, 0.0); // Move forward again with 70% speed
    } else {
      m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
    }
  }

  private void runDefaultAuto() {
    if (m_timer.get() < 4.0) {
      m_robotDrive.arcadeDrive(1.0, 0.0); // Move forward with 100% speed
    } else {
      m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
    }
  }

  private void updateField2d() {
    // Updating the field visualization, I have  really good odometry
    m_field.setRobotPose(m_field.getRobotPose());
  }
}
