// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ZoneConstants;
import frc.utils.Zone;


/** Add your docs here. */
public class RobotState {

    private Pose2d m_robotPose2d;

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

    public Pose2d getRobotPose(){
        return m_robotPose2d;
    }

    public Alliance getAlliance(){
        Alliance alliance;
        if(!DriverStation.getAlliance().isPresent()) alliance = Alliance.Blue;
        else alliance = DriverStation.getAlliance().get();
        return alliance;
    }

    public boolean inWing(){
        if(getAlliance() == Alliance.Blue) return ZoneConstants.BlueWing.inZone(m_robotPose2d);
        else return ZoneConstants.RedWing.inZone(m_robotPose2d);
    }

    public boolean inStage(){
        if(getAlliance() == Alliance.Blue) return ZoneConstants.BlueStage.inZone(m_robotPose2d);
        else return ZoneConstants.RedStage.inZone(m_robotPose2d);
    }

    public boolean inSpeaker(){
        if(getAlliance()== Alliance.Blue) return ZoneConstants.BlueSpeaker.inZone(m_robotPose2d);
        else return ZoneConstants.RedSpeaker.inZone(m_robotPose2d);
    }

    public boolean inAmp(){
        if(getAlliance() == Alliance.Blue) return ZoneConstants.BlueAmp.inZone(m_robotPose2d);
        else return ZoneConstants.RedAmp.inZone(m_robotPose2d);
    }

    public boolean inSource(){
        if(getAlliance() == Alliance.Blue) return ZoneConstants.BlueSource.inZone(m_robotPose2d);
        else return ZoneConstants.RedSource.inZone(m_robotPose2d);
    }

}
