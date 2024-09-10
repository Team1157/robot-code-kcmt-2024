package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Robot extends TimedRobot {
  private final PWMTalonSRX m_leftMotor = new PWMTalonSRX(0);
  private final PWMTalonSRX m_rightMotor = new PWMTalonSRX(2);
  private final PWMTalonSRX m_leftFollower = new PWMTalonSRX(1);
  private final PWMTalonSRX m_rightFollower = new PWMTalonSRX(3);
  private final DifferentialDrive m_robotDrive =
  new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  private final XboxController m_driverController = new XboxController(0);
  private BuiltInAccelerometer m_accelerometer;
  private NetworkTableInstance m_ntInstance;
  private NetworkTable m_dashboardTable;
  private NetworkTableEntry m_accelXEntry;
  private NetworkTableEntry m_accelYEntry;
  private NetworkTableEntry m_accelZEntry;
  private static final String kDefaultAuto = "auto0";
  private static final String kCustomAuto1 = "auto1";
  private static final String kCustomAuto2 = "auto2";
  private static final String kCustomAuto3 = "auto3";
  private String m_autoSelected;
  private final SendableChooser < String > m_chooser = new SendableChooser < > ();
  private final Timer m_timer = new Timer();

  // Field2d object for field visualization
  private Field2d m_field;

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("auto0", kDefaultAuto);
    m_chooser.addOption("auto1", kCustomAuto1);
    m_chooser.addOption("auto2", kCustomAuto2);
    m_chooser.addOption("auto3", kCustomAuto3);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_leftMotor.addFollower(m_leftFollower);
    m_rightMotor.addFollower(m_rightFollower);
    m_rightMotor.setInverted(true);
    m_accelerometer = new BuiltInAccelerometer();

    // Initialize NetworkTables
    m_ntInstance = NetworkTableInstance.getDefault();
    m_dashboardTable = m_ntInstance.getTable("SmartDashboard");

    // Create NetworkTable entries
    m_accelXEntry = m_dashboardTable.getEntry("AccelX");
    m_accelYEntry = m_dashboardTable.getEntry("AccelY");
    m_accelZEntry = m_dashboardTable.getEntry("AccelZ");
    m_dashboardTable.getEntry("TargetSpeed");

    // Initialize the Field2d object
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
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
      // Drive forward for 2 seconds, then turn right for 1 second
      if (m_timer.get() < 2.0) {
        m_robotDrive.arcadeDrive(0.5, 0.0); // Move forward with 50% speed
      } else if (m_timer.get() < 3.0) {
        m_robotDrive.arcadeDrive(0.0, 0.5); // Turn right with 50% speed
      } else {
        m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
      }
      break;
    case kCustomAuto2:
      // Drive forward for 3 seconds, then turn left for 1.5 seconds
      if (m_timer.get() < 3.0) {
        m_robotDrive.arcadeDrive(0.6, 0.0); // Move forward with 60% speed
      } else if (m_timer.get() < 4.5) {
        m_robotDrive.arcadeDrive(0.0, -0.6); // Turn left with 60% speed
      } else {
        m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
      }
      break;
    case kCustomAuto3:
      // Drive forward for 1.5 seconds, turn right for 2 seconds, and then drive forward again
      if (m_timer.get() < 1.5) {
        m_robotDrive.arcadeDrive(0.7, 0.0); // Move forward with 70% speed
      } else if (m_timer.get() < 3.5) {
        m_robotDrive.arcadeDrive(0.0, 0.7); // Turn right with 70% speed
      } else if (m_timer.get() < 5.0) {
        m_robotDrive.arcadeDrive(0.7, 0.0); // Move forward again with 70% speed
      } else {
        m_robotDrive.arcadeDrive(0.0, 0.0); // Stop
      }
      break;
    case kDefaultAuto:
    default:
      // Drive forward for 4 seconds, just for testing 
      if (m_timer.get() < 4.0) {
        m_robotDrive.arcadeDrive(1.0, 0.0); // Move forward with 100% speed
      } else {
        m_robotDrive.arcadeDrive(0.0, 0.0); // Stop after 4 seconds
      }
      break;
    }

    // Update the Field2d position in autonomous
    updateField2d();
  }

  private void updateField2d() {
    // Get the current pose from the drivetrain and update the Field2d object
    m_field.setRobotPose(m_field.getRobotPose());
  }
}
