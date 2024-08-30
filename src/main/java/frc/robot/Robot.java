package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
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
    @Override
    public void robotInit() {
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
    public void teleopPeriodic() {
        double forwardBackward = -m_joystick.getRawAxis(1); // Forward/backward with left stick Y-axis (index 1)
        double leftRight = m_joystick.getRawAxis(0);       // Left/right turning with left stick X-axis (index 0)
        m_drivetrain.arcadeDrive(forwardBackward, leftRight);
        m_accelXEntry.setDouble(m_accelerometer.getX());        // X-axis acceleration
        m_accelYEntry.setDouble(m_accelerometer.getY());        // Y-axis acceleration
        m_accelZEntry.setDouble(m_accelerometer.getZ());        // Z-axis acceleration
    }
}
