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
import edu.wpi.first.wpilibj.Timer;
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

    private PhotonCamera[] allCameras = new PhotonCamera[4];

    

    public AprilTagFieldLayout layout;
    private PhotonCamera  m_targetCam = null;
    private Timer m_targetTimer = new Timer();
    private double m_lockTimeSec = 0.2;

    public static enum Side {
        FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
    }

    public PhotonCameraWrapper() {
        photonCameraFrontLeft = new PhotonCamera(CameraConstants.photonCameraNameFrontLeft);
        photonCameraFrontRight = new PhotonCamera(CameraConstants.photonCameraNameFrontRight);
        photonCameraRearLeft = new PhotonCamera(CameraConstants.photonCameraNameRearLeft);
        photonCameraRearRight = new PhotonCamera(CameraConstants.photonCameraNameRearRight);

        allCameras[0] = photonCameraFrontLeft;
        allCameras[1] = photonCameraFrontRight;
        allCameras[2] = photonCameraRearLeft;
        allCameras[3] = photonCameraRearRight;
        
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

    public Optional<Double> getYawToTarget(int id){
        
        //we're using a camera that was already locked on
        if (null != m_targetCam){  
            var result = m_targetCam.getLatestResult();
            for (var target : result.getTargets()){
                if (id == target.getFiducialId()){
                    m_targetTimer.restart();
                    return Optional.of(Double.valueOf(target.getYaw()));
                }
            }
            //we didn't find it with the locked camera.
            //if the timer has expired, go back to all cams.
            if(m_targetTimer.hasElapsed(m_lockTimeSec)){
                m_targetCam = null;
                m_targetTimer.stop();
            }
        } else { //no pre locked camera.  loop through them all.
            for (PhotonCamera cam : allCameras){
                var newResult = cam.getLatestResult();
                for (var target : newResult.getTargets()){
                    if (id == target.getFiducialId()){
                        m_targetCam = cam;
                        m_targetTimer.restart();
                        return Optional.of(Double.valueOf(target.getYaw()));
                    }
                }
            }
        }
        
        //no targets found anywhere.
        m_targetCam = null;
        m_targetTimer.stop();
        return Optional.empty();

    }

}