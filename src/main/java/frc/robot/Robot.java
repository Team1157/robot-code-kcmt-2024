package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private Drivetrain m_drivetrain;
    private Joystick m_joystick;

    @Override
    public void robotInit() {
        m_drivetrain = new Drivetrain();
        m_joystick = new Joystick(0);  // Use joystick on port 0
    }

    @Override
    public void teleopPeriodic() {
        double forwardBackward = -m_joystick.getRawAxis(1); // Forward/backward with left stick Y-axis (index 1)
        double leftRight = m_joystick.getRawAxis(0);       // Left/right turning with left stick X-axis (index 0)
        m_drivetrain.arcadeDrive(forwardBackward, leftRight);
    }
}
