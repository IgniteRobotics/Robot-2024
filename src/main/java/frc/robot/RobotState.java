// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class RobotState {

    private Pose2d m_robotPose2d;
    private Pose2d m_speakerPose2d;
    private int m_speakerID = 0;
    private boolean m_hasNote = false;
    private double m_distanceToSpeaker;

    private static RobotState single_instance = null;

    private RobotState() {
    }

    public static synchronized RobotState getInstance()
    {
        if (single_instance == null)
            single_instance = new RobotState();
        
        return single_instance;
    }

    public synchronized void setRobotPose(Pose2d pose){
        m_robotPose2d = pose;

    }

    public synchronized void setHasNote(boolean flag){
        m_hasNote = flag;
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
        return m_distanceToSpeaker;
    }

    public void setDistanceToSpeaker(double d) {
        m_distanceToSpeaker = d;
    }

    public boolean hasNote() {
        return m_hasNote;
    }

}
