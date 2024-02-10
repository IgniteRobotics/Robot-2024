package frc.robot.subsystems.drive;
import java.util.List;

import java.io.UncheckedIOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.CameraConstants;

public class PhotonCameraWrapper {
    public PhotonCamera photonCameraFrontLeft;
    public PhotonPoseEstimator photonPoseEstimatorFrontLeft;
    public PhotonCamera photonCameraFrontRight;
    public PhotonPoseEstimator photonPoseEstimatorFrontRight;
    public PhotonCamera photonCameraRearLeft;
    public PhotonPoseEstimator photonPoseEstimatorRearLeft;
    public PhotonCamera photonCameraRearRight;
    public PhotonPoseEstimator photonPoseEstimatorRearRight;
    

    public AprilTagFieldLayout layout;

    public static enum Side {
        FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
    }

    public PhotonCameraWrapper() {
        photonCameraFrontLeft = new PhotonCamera(CameraConstants.photonCameraNameFrontLeft);
        photonCameraFrontRight = new PhotonCamera(CameraConstants.photonCameraNameFrontRight);
        photonCameraRearLeft = new PhotonCamera(CameraConstants.photonCameraNameRearLeft);
        photonCameraRearRight = new PhotonCamera(CameraConstants.photonCameraNameRearRight);
        
        try {
            layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (UncheckedIOException e) {
            e.printStackTrace();
        }

        //TODO: investigate PNP on the co-proc.
        photonPoseEstimatorFrontLeft = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraFrontLeft, CameraConstants.photonCameraTransformFrontLeft);
        photonPoseEstimatorFrontRight = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraFrontRight, CameraConstants.photonCameraTransformFrontRight);
        photonPoseEstimatorRearLeft = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraRearLeft, CameraConstants.photonCameraTransformRearLeft);
        photonPoseEstimatorRearRight = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCameraRearRight, CameraConstants.photonCameraTransformRearRight);

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, Side side) {
        if(side == Side.FRONT_RIGHT) {
            photonPoseEstimatorFrontRight.setReferencePose(prevEstimatedRobotPose);
            
            return photonPoseEstimatorFrontRight.update();
        } else if (side == Side.FRONT_LEFT){
            photonPoseEstimatorFrontLeft.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimatorFrontLeft.update();
        } else if (side == Side.REAR_RIGHT){
            photonPoseEstimatorRearRight.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimatorRearRight.update();
        } else {
            photonPoseEstimatorRearLeft.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimatorRearLeft.update();
        } 
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        return getEstimatedGlobalPose(prevEstimatedRobotPose, Side.FRONT_LEFT);
    }
}