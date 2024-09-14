package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonVersion;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.*;;

public class uwu {
       // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 208
    public static final double targetWidth =
    0.87884;
    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    public static final double targetHeight =
    0.4318;

    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    public static final double kFarTgtXPos = 16.4592;
    public static final double kFarTgtYPos =2.3931;
    public static final double kFarTgtZPos =1.4645;

    public static final Pose3d kFarTargetPose =
            new Pose3d(
                    new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
                    new Rotation3d(0.0, 0.0, 3.14));
    PhotonCamera camera = new PhotonCamera("uwuCamera");
    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(AprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
}
