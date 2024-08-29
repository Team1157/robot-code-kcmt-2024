package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private Drivetrain m_drivetrain;
    private Joystick m_leftStick;
    private Joystick m_rightStick;

    @Override
    public void robotInit() {
        m_drivetrain = new Drivetrain();
        m_leftStick = new Joystick(0);  // Left stick for forward/backward
        m_rightStick = new Joystick(1); // Right stick for left/right turning
    }

    @Override
    public void teleopPeriodic() {
        double forwardBackward = -m_leftStick.getY(); // Forward/backward with left stick Y-axis
        double leftRight = m_rightStick.getX();       // Left/right turning with right stick X-axis
        m_drivetrain.arcadeDrive(forwardBackward, leftRight);
    }
}
