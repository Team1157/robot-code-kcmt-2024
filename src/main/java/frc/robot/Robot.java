package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private static final String kCustomAuto = "auto1";
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
    m_chooser.addOption("auto1", kCustomAuto);
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
    case kCustomAuto:
      // auto1 code idk something like a drive to centerline and disrupt other bots, we win at chicken
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

    // Update the Field2d position in autonomous so we know where its going idk some of the driverstations have a bad view from behind the line
    updateField2d();
  }

  private void updateField2d() {
    // Get the current pose from the drivetrain and update the Field2d object
    m_field.setRobotPose(m_field.getRobotPose());
  }
}
