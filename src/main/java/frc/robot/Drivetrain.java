package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Drivetrain {
    private DifferentialDrive m_robotDrive;

    // Motors
    private final PWMVictorSPX m_leftMotor1 = new PWMVictorSPX(0);
    private final PWMSparkMax m_leftMotor2 = new PWMSparkMax(1);
    private final PWMSparkMax m_rightMotor1 = new PWMSparkMax(2);
    private final PWMVictorSPX m_rightMotor2 = new PWMVictorSPX(3);

    public Drivetrain() {
        // Group the motors
        MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
        MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);

        // Invert the right side motors (depends on wiring, UNTESTED MIGHT HAVE TO FLIP WHICH ONE AND IF AT ALL)
        rightMotors.setInverted(true);

        // Setup DifferentialDrive
        m_robotDrive = new DifferentialDrive(leftMotors, rightMotors);

        // Register motors
        SendableRegistry.addChild(m_robotDrive, m_leftMotor1);
        SendableRegistry.addChild(m_robotDrive, m_leftMotor2);
        SendableRegistry.addChild(m_robotDrive, m_rightMotor1);
        SendableRegistry.addChild(m_robotDrive, m_rightMotor2);
    }

    public DifferentialDrive getDrive() {
        return m_robotDrive;
    }

    public double arcadeDrive(double forwardBackward, double leftRight) {
        m_robotDrive.arcadeDrive(forwardBackward, leftRight);
        return leftRight;
    }
}
