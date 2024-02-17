// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Add your docs here. */
public class DestinationUtil {
    private final static Pose2d BASourceP = new Pose2d(40, 10, new Rotation2d(3*Math.PI/2));
    private final static Pose2d RASourceP = new Pose2d(10, 10, new Rotation2d(3*Math.PI/2));

    private final static Pose2d BASpeakerP = new Pose2d(2.9, 5.56, new Rotation2d(0));
    private final static Pose2d RASpeakerP = new Pose2d(13.66, 5.56, new Rotation2d(Math.PI));

    private final static Pose2d BAAmpP = new Pose2d(1.78, 5.75, new Rotation2d(Math.PI/2));
    private final static Pose2d RAAmpP = new Pose2d(14.72, 5.75, new Rotation2d(Math.PI/2));
    
    public Pose2d SourceFinder(Optional<Alliance> temp)
    {
        Alliance alliance;
        if(temp.isPresent()) alliance = temp.get();
        else alliance = Alliance.Blue;

        if(alliance == Alliance.Red) return RASourceP;
        else return BASourceP;
    }

    public Pose2d SpeakerFinder(Optional<Alliance> temp)
    {
        Alliance alliance;
        if(temp.isPresent()) alliance = temp.get();
        else alliance = Alliance.Blue;

        if(alliance == Alliance.Red) return RASpeakerP;
        else return BASpeakerP;
    }

    public Pose2d AmpFinder(Optional<Alliance> temp)
    {
        Alliance alliance;
        if(temp.isPresent()) alliance = temp.get();
        else alliance = Alliance.Blue;

        if(alliance == Alliance.Red) return RAAmpP;
        else return BAAmpP;
    }
}
