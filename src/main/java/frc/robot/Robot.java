package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.util.sendable.SendableRegistry;

public class Robot extends TimedRobot {
    private ADXRS450_Gyro gyro;
    private DifferentialDriveOdometry m_odometry;
    private DifferentialDrive m_robotDrive;
    private XboxController m_driverController;
    private PowerDistribution m_pdp;
    private PWMTalonSRX m_leftMotor, m_rightMotor, m_leftFollower, m_rightFollower;
    private BuiltInAccelerometer m_accelerometer;
    private NetworkTableInstance m_ntInstance;
    private NetworkTable m_dashboardTable;
    private NetworkTableEntry m_visionTargetXEntry, m_visionTargetYEntry, m_visionTargetRotationEntry;
    private NetworkTableEntry m_accelXEntry, m_accelYEntry, m_accelZEntry;
    private NetworkTableEntry m_leftMotorOutputEntry, m_rightMotorOutputEntry;
    private NetworkTableEntry m_leftFollowerOutputEntry, m_rightFollowerOutputEntry;
    private NetworkTableEntry m_timerEntry, m_fieldPositionEntry, m_drivemode, m_slowmode;
    private Pose2d m_visionTargetPose = new Pose2d(); // Initialize Pose2d with default values
    private final Timer m_timer = new Timer();
    private final Field2d m_field = new Field2d();

    // Constants for motor ports and track width
    private static final int LEFT_MOTOR_PORT = 0;
    private static final int LEFT_FOLLOWER_PORT = 1;
    private static final int RIGHT_MOTOR_PORT = 2;
    private static final int RIGHT_FOLLOWER_PORT = 3;
    private final double kTrackWidthMeters = 0.69; // Example track width (meters)

    @Override
    public void robotInit() {
        // Initialize and calibrate gyro
        gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
        gyro.calibrate();

        // Initialize motor controllers and other hardware
        m_leftMotor = new PWMTalonSRX(LEFT_MOTOR_PORT);
        m_rightMotor = new PWMTalonSRX(RIGHT_MOTOR_PORT);
        m_leftFollower = new PWMTalonSRX(LEFT_FOLLOWER_PORT);
        m_rightFollower = new PWMTalonSRX(RIGHT_FOLLOWER_PORT);
        m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        m_driverController = new XboxController(0);
        m_pdp = new PowerDistribution(0, ModuleType.kCTRE);

        // Configure motors
        configureMotors();

        // Initialize NetworkTables
        initializeNetworkTables();

        // Initialize odometry
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), kTrackWidthMeters, kTrackWidthMeters);

        // Add field visualization to SmartDashboard
        SmartDashboard.putData("Field", m_field);
    }

    private void configureMotors() {
        m_leftMotor.addFollower(m_leftFollower);
        m_rightMotor.addFollower(m_rightFollower);
        m_rightMotor.setInverted(true); // Invert right motors so it won't just go in circles
        m_accelerometer = new BuiltInAccelerometer();
    }

    private void initializeNetworkTables() {
        m_ntInstance = NetworkTableInstance.getDefault();
        m_dashboardTable = m_ntInstance.getTable("photonvision/uwucamera"); // Replace with your actual camera table path

        // Initialize entries for pose components
        m_visionTargetXEntry = m_dashboardTable.getEntry("targetPoseX");
        m_visionTargetYEntry = m_dashboardTable.getEntry("targetPoseY");
        m_visionTargetRotationEntry = m_dashboardTable.getEntry("targetPoseRotation");

        // Other initializations
        m_accelXEntry = m_dashboardTable.getEntry("AccelX");
        m_accelYEntry = m_dashboardTable.getEntry("AccelY");
        m_accelZEntry = m_dashboardTable.getEntry("AccelZ");
        m_leftMotorOutputEntry = m_dashboardTable.getEntry("LeftMotorOutput");
        m_rightMotorOutputEntry = m_dashboardTable.getEntry("RightMotorOutput");
        m_leftFollowerOutputEntry = m_dashboardTable.getEntry("LeftFollowerOutput");
        m_rightFollowerOutputEntry = m_dashboardTable.getEntry("RightFollowerOutput");
        m_timerEntry = m_dashboardTable.getEntry("MatchTime");
        m_fieldPositionEntry = m_dashboardTable.getEntry("FieldPosition");
        m_drivemode = m_dashboardTable.getEntry("DriveMode");
        m_slowmode = m_dashboardTable.getEntry("SlowMode");
    }

    private void updatePoseFromNetworkTables() {
        NetworkTableEntry poseEntry = m_dashboardTable.getEntry("targetPose");
        double[] poseArray = poseEntry.getDoubleArray(new double[0]);

        // Debug: Print the length and contents of the pose array
        System.out.println("Pose array length: " + poseArray.length);

        if (poseArray.length >= 7) {
            // Extract pose values from the targetPose array
            double x = poseArray[0];
            double y = poseArray[1];
            double z = poseArray[2]; // Assuming z is not used for 2D calculations
            double qw = poseArray[3];
            double qx = poseArray[4];
            double qy = poseArray[5];
            double qz = poseArray[6];

            // Create Rotation2d from quaternion (qw, qx, qy, qz)
            Rotation2d targetRotation = new Rotation2d(Math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz));
            
            // Create Pose2d from x, y, and the targetRotation
            Pose2d targetPose = new Pose2d(x, y, targetRotation);

            // Convert the target pose to field-relative
            Pose2d robotPose = m_odometry.getPoseMeters();
            Pose2d fieldRelativePose = robotPose.relativeTo(targetPose);

            // Update the Field2d visualization
            m_field.setRobotPose(fieldRelativePose);

            // Set the vision target pose
            m_visionTargetPose = fieldRelativePose;
        } else {
            // Handle cases where poseArray does not have the expected length
            System.err.println("Pose array does not contain enough elements.");
            m_visionTargetPose = new Pose2d(); // Reset to default Pose2d
        }
    }

    @Override
    public void robotPeriodic() {
        // Update pose data
        updatePoseFromNetworkTables();

        // Display current pose on SmartDashboard
        SmartDashboard.putString("Field/Robot", m_field.getRobotPose().toString());
        SmartDashboard.putString("Vision Target Pose", m_visionTargetPose.toString());
    }

    @Override
    public void teleopPeriodic() {
        // Read controller inputs for field-relative drive
        double forwardSpeed = -m_driverController.getLeftY();
        double strafeSpeed = m_driverController.getLeftX();
        double rotationSpeed = -m_driverController.getRawAxis(5);

        // Get the current gyro angle
        double gyroAngle = gyro.getAngle();

        // Apply field-relative drive
        driveFieldRelative(forwardSpeed, strafeSpeed, rotationSpeed, gyroAngle);

        // Reset gyro and calibrate gyro based on button inputs
        if (m_driverController.getRawButton(8)) {
            gyro.reset();
        }
        if (m_driverController.getRawButton(2)) {
            gyro.calibrate();
        }
    }

    private void driveFieldRelative(double forward, double strafe, double rotation, double gyroAngle) {
        // Convert gyro angle to radians
        double gyroRadians = Math.toRadians(gyroAngle);
        
        m_drivemode.setString("Field centric :)");

        // Apply field-relative transformations
        double temp = forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
        strafe = -forward * Math.sin(gyroRadians) + strafe * Math.cos(gyroRadians);
        forward = temp;

        // Adjust inputs for differential drive
        double leftMotorOutput = forward + rotation;
        double rightMotorOutput = forward - rotation;

        // Make bot fasttttt
        leftMotorOutput *= 2;
        rightMotorOutput *= 1.8;

        // Slow mode
        if (m_driverController.getRawButton(6)) {
            m_slowmode.setBoolean(true);
            leftMotorOutput *= 0.5;
            rightMotorOutput *= 0.5;
        } else {
            m_slowmode.setBoolean(false);
        }

        // Drive the robot
        m_robotDrive.tankDrive(leftMotorOutput, rightMotorOutput);

        // Update motor outputs to SmartDashboard
        m_leftMotorOutputEntry.setDouble(leftMotorOutput);
        m_rightMotorOutputEntry.setDouble(rightMotorOutput);
    }
}
