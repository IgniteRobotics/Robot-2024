// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class RobotState {

    private Pose2d m_robotPose2d;
    private Pose2d m_speakerPose2d;
    private int m_speakerID = 0;
    private boolean m_hasNote = false;
    private Alliance m_Alliance = null;

    private static RobotState single_instance = null;

    private RobotState() {
        //default for safety.  NPEs are really bad
        m_robotPose2d = new Pose2d(); 
        m_Alliance = Alliance.Blue;
    }

    public static synchronized RobotState getInstance()
    {
        if (single_instance == null)
            single_instance = new RobotState();
        
        return single_instance;
    }

    public synchronized void setAlliance(Alliance a){
        m_Alliance = a;
    }

    public Pose2d getAmpSideShotPose(){
        if (m_Alliance == Alliance.Red){
            return Constants.ShooterConstants.RED_SPK_AMP_SHOT;
        } else {
            return Constants.ShooterConstants.BLUE_SPK_AMP_SHOT;
        }
    }

    public Pose2d getMidShotPose(){
        if (m_Alliance == Alliance.Red){
            return Constants.ShooterConstants.RED_SPK_MID_SHOT;
        } else {
            return Constants.ShooterConstants.BLUE_SPK_MID_SHOT;
        }
    }

    public Pose2d getSourceSideShotPose(){
        if (m_Alliance == Alliance.Red){
            return Constants.ShooterConstants.RED_SPK_SRC_SHOT;
        } else {
            return Constants.ShooterConstants.BLUE_SPK_SRC_SHOT;
        }
    }

    public Pose2d getClosestShotPose(){
        if (m_Alliance == Alliance.Red){
            return m_robotPose2d.nearest(Constants.ShooterConstants.RED_SHOT_POSES);
        } else {
            return m_robotPose2d.nearest(Constants.ShooterConstants.BLUE_SHOT_POSES);
        }
    }


    public synchronized void setRobotPose(Pose2d pose){
        m_robotPose2d = pose;

    }

    public Pose2d getRobotPose(){
        return m_robotPose2d;
    }

    public synchronized void setSpeakerPose(Pose2d pose, int id){
        m_speakerPose2d = pose;
        m_speakerID = id;
    }

    public Pose2d getSpeakerPose2d(){
        return m_robotPose2d;
    }

    public int getSpeakerID(){
        return m_speakerID;
    }

    public double getDistancetoSpeaker(){
        if (null == m_robotPose2d || null == m_speakerPose2d) return 0.0;
        return PhotonUtils.getDistanceToPose(m_robotPose2d, m_speakerPose2d);
    }

    public synchronized void setHasNote(boolean flag){
        m_hasNote = flag;
    }

    public boolean hasNote() {
        return m_hasNote;
    }

}
