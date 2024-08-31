package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
    private Drivetrain m_drivetrain;
    private Joystick m_joystick;
    private BuiltInAccelerometer m_accelerometer;
    private NetworkTableInstance m_ntInstance;
    private NetworkTable m_dashboardTable;
    private NetworkTableEntry m_accelXEntry;
    private NetworkTableEntry m_accelYEntry;
    private NetworkTableEntry m_accelZEntry;
    private static final String kDefaultAuto = "auto0";
    private static final String kCustomAuto = "auto1";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final Timer m_timer = new Timer();
    
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("auto0", kDefaultAuto);
        m_chooser.addOption("auto1", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        m_drivetrain = new Drivetrain();
        m_joystick = new Joystick(0);  // Use joystick on port 0
        m_accelerometer = new BuiltInAccelerometer();

        // Initialize NetworkTables
        m_ntInstance = NetworkTableInstance.getDefault();
        m_dashboardTable = m_ntInstance.getTable("SmartDashboard");

        // Create NetworkTable entries
        m_accelXEntry = m_dashboardTable.getEntry("AccelX");
        m_accelYEntry = m_dashboardTable.getEntry("AccelY");
        m_accelZEntry = m_dashboardTable.getEntry("AccelZ");
        m_dashboardTable.getEntry("TargetSpeed");
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
                    m_drivetrain.arcadeDrive(1.0, 0.0); // Move forward with 50% speed
                } else {
                    m_drivetrain.arcadeDrive(1.0, 0.0); // Stop after 4 seconds
                }
                break;
        }
    }
    
    @Override
    public void teleopPeriodic() {
        double forwardBackward = -m_joystick.getRawAxis(1); // Forward/backward with left stick Y-axis (index 1)
        double leftRight = m_joystick.getRawAxis(0);       // Left/right turning with left stick X-axis (index 0)
        m_drivetrain.arcadeDrive(forwardBackward, leftRight);
        m_accelXEntry.setDouble(m_accelerometer.getX());        // X-axis acceleration
        m_accelYEntry.setDouble(m_accelerometer.getY());        // Y-axis acceleration
        m_accelZEntry.setDouble(m_accelerometer.getZ());        // Z-axis acceleration
    }
}
