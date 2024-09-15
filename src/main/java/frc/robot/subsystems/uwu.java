package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;

public class UwU {
    private static final double CAMERA_HEIGHT_METERS = 0.1;
    private static final double TARGET_HEIGHT_METERS = 1;
    private static final double CAMERA_PITCH_RADIANS = 0;
    private static final double GOAL_RANGE_METERS = 3.0; // Desired goal range

    private final PhotonCamera camera = new PhotonCamera("uwuCamera");
    private final XboxController xboxController = new XboxController(0); // Example controller port
    private final PIDController controller = new PIDController(1.0, 0.0, 0.0); // Example PID parameters
    
    // Initialize Transform3d objects with default values
    private final Transform3d cameraToTarget = new Transform3d();
    private final Transform3d cameraToRobot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)
    );

    public UwU() {
        // Constructor implementation here
    }

    public void periodic() {
        if (xboxController.getRawButton(1)) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            PhotonPipelineResult result = camera.getLatestResult();

            if (result.hasTargets()) {
                // Get the best target from the result
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                
                // Define the pose of the AprilTag relative to the field (example values)
                Pose3d fieldRelativeTagPose = new Pose3d(
                    new Translation3d(1.0, 2.0, 0.0), // Replace with actual values
                    new Rotation3d(0.0, 0.0, 0.0)    // Replace with actual values
                );

                // Ensure the cameraToTarget and fieldRelativeTagPose are valid
                if (cameraToTarget != null && fieldRelativeTagPose != null) {
                    // Estimate the robot pose relative to the field
                    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        cameraToTarget,
                        fieldRelativeTagPose,
                        cameraToRobot
                    );

                    // Calculate range from the estimated pose
                    double pitch = bestTarget.getPitch();
                    if (pitch != 0.0) { // Ensure pitch is not zero to avoid invalid calculations
                        double range = PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            pitch
                        );

                        // Use this range as the measurement we give to the PID controller
                        // -1.0 required to ensure positive PID controller effort _increases_ range
                        double forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);

                        // Use forwardSpeed to control the robot's movement
                        System.out.println("Forward Speed: " + forwardSpeed);
                    } else {
                        System.out.println("Invalid pitch value.");
                    }
                } else {
                    System.out.println("cameraToTarget or fieldRelativeTagPose is null.");
                }
            } else {
                // Handle the case where no targets are found
                System.out.println("No targets found.");
            }
        }
    }
}
