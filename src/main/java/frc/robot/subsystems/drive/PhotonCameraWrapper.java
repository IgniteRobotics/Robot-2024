package frc.robot.subsystems.drive;
import java.util.List;

import java.io.UncheckedIOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CameraConstants;

public class PhotonCameraWrapper {

    public class TargetInfo{
        private double yaw;
        private double distance;

        public TargetInfo(double distance, double yaw){
            this.distance = distance;
            this.yaw = yaw;
        }

        public double getYaw() {
            return yaw;
        }

        public void setYaw(double yaw) {
            this.yaw = yaw;
        }

        public double getDistance() {
            return distance;
        }

        public void setDistance(double distance) {
            this.distance = distance;
        }
    }






    public PhotonCamera photonCameraFrontLeft;
    public PhotonPoseEstimator photonPoseEstimatorFrontLeft;
    public PhotonCamera photonCameraFrontRight;
    public PhotonPoseEstimator photonPoseEstimatorFrontRight;
    public PhotonCamera photonCameraRearLeft;
    public PhotonPoseEstimator photonPoseEstimatorRearLeft;
    public PhotonCamera photonCameraRearRight;
    public PhotonPoseEstimator photonPoseEstimatorRearRight;

    private PhotonCamera[] allCameras = new PhotonCamera[4];
    private PhotonPoseEstimator[] allEstimators = new PhotonPoseEstimator[4];

    

    public AprilTagFieldLayout layout;
    private PhotonCamera  m_targetCam = null;
    private PhotonPoseEstimator m_targetEstimator = null;
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
        
        allEstimators[0] = photonPoseEstimatorFrontLeft;
        allEstimators[1] = photonPoseEstimatorFrontRight;
        allEstimators[2] = photonPoseEstimatorRearLeft;
        allEstimators[3] = photonPoseEstimatorRearRight;

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

    public Optional<TargetInfo> seekTarget(int id){
        
        //we're using a camera that was already locked on
        if (null != m_targetCam){  
            var result = m_targetCam.getLatestResult();
            Optional<PhotonTrackedTarget> target = lookForTarget(result, id);
            if (target.isPresent()){
                m_targetTimer.restart();
                //return Optional.of(new TargetInfo(getDistanceFromTransform3d(target.get().getBestCameraToTarget()), target.get().getYaw()));
                return Optional.of(buildTargetInfo(target.get().getBestCameraToTarget(), m_targetEstimator.getRobotToCameraTransform()));
            }

            //we didn't find it with the locked camera.
            //if the timer has expired, go back to all cams.
            if(m_targetTimer.hasElapsed(m_lockTimeSec)){
                m_targetCam = null;
                m_targetEstimator = null;
                m_targetTimer.stop();
            }
        } else { //no pre locked camera.  loop through them all.
            for (int i = 0; i < allCameras.length; i++){
                var newResult = allCameras[i].getLatestResult();
                Optional<PhotonTrackedTarget> target = lookForTarget(newResult, id);
                if (target.isPresent()){
                    m_targetCam = allCameras[i];
                    m_targetEstimator = allEstimators[i];
                    m_targetTimer.restart();
                    //return Optional.of(new TargetInfo(getDistanceFromTransform3d(target.get().getBestCameraToTarget()), target.get().getYaw()));
                    return Optional.of(buildTargetInfo(target.get().getBestCameraToTarget(), m_targetEstimator.getRobotToCameraTransform()));
                }
            }
        }
        
        //no targets found anywhere.
        m_targetCam = null;
        m_targetEstimator = null;
        m_targetTimer.stop();
        return Optional.empty();

    }

    private Optional<PhotonTrackedTarget> lookForTarget(PhotonPipelineResult result, int targetId){
        for (var target : result.getTargets()){
                if (targetId == target.getFiducialId()){
                    return Optional.of(target) ;
                }
            }
        return Optional.empty();
    }

    private double getDistanceFromTransform3d(Transform3d t){
        return Math.sqrt(
                Math.pow(t.getX(), 2) + 
                Math.pow(t.getY(), 2)
        );
    }

    private TargetInfo buildTargetInfo(Transform3d cam2Target, Transform3d robot2Cam){
        Transform3d robot2Target = cam2Target.plus(robot2Cam);
        return new TargetInfo(getDistanceFromTransform3d(robot2Target), 
            robot2Target.getRotation().getAngle());
    }

    public void unlockTargeting(){
        m_targetCam = null;
        m_targetEstimator = null;
        m_targetTimer.stop();
    }

}