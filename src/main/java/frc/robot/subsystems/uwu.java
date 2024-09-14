package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonVersion;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class uwu {
    PhotonCamera camera = new PhotonCamera("uwuCamera");
    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(AprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
}
