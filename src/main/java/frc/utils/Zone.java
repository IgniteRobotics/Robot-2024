// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class Zone {


    private  double x1, y1, x2, y2;
    public Zone(double x1, double y1, double x2, double y2){
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
    }

    public boolean inZone(Pose2d pose)
    {
        if(pose.getX() > x1 && pose.getY() > y1 && pose.getX() < x2 && pose.getY() < y2) return true;
        else return false;
    }
}

