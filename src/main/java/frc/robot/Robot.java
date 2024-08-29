// This is where our code actually goes
package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  // all the motors
  private final PWMVictorSPX m_leftMotor1 = new PWMVictorSPX(0);
  private final PWMSparkMax m_leftMotor2 = new PWMSparkMax(1);
  private final PWMSparkMax m_rightMotor1 = new PWMSparkMax(2);
  private final PWMVictorSPX m_rightMotor2 = new PWMVictorSPX(3);

  @Override
  public void robotInit() {
    // Group the motors
    MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);

    // Register motors
    SendableRegistry.addChild(m_robotDrive, m_leftMotor1);
    SendableRegistry.addChild(m_robotDrive, m_leftMotor2);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor1);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor2);

    // Invert the right side motors (depends on wiring)
    rightMotors.setInverted(true);

    // Setup DifferentialDrive
    m_robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    m_leftStick = new Joystick(0);  // Left stick for forward/backward
    m_rightStick = new Joystick(1); // Right stick for left/right turning like a drone controller thingy idk its more intuitive for me
  }

  @Override
  public void teleopPeriodic() {
    double forwardBackward = -m_leftStick.getY(); // Forward/backward with left stick Y-axis
    double leftRight = m_rightStick.getX();       // Left/right turning with right stick X-axis
    m_robotDrive.arcadeDrive(forwardBackward, leftRight);
  }
}
