// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

/** Add your docs here. */
public class Timer {
    long timer;

    public long getTimer(){
        return timer;
    }

    public void resetTimer(){
        timer = 0;
    }

    public boolean timePassed(int milliseconds){
        return System.currentTimeMillis() > timer + milliseconds;
    }
}
