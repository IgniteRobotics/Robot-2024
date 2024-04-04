package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;


public class PhotonCameraWrapperTest {

    @Test
    public void testAngleCalculations() {
        PhotonCameraWrapper pcw = new PhotonCameraWrapper();

        /*
         * double yawToTargetDegrees, 
         * double distanceToTargetMeters, 
         * double cameraYawOffset, 
         * double cameraYOffsetMeters
         */
        assertEquals(3.23, pcw.calculateTargetInfo(
            18.5, 
            3.2, 
            -15.0, 
            0.271).getDistance());
        assertEquals(-8.31, pcw.calculateTargetInfo(
            18.5, 
            3.2, 
            -15.0, 
            0.271).getYaw());
        
        assertEquals(-5.16, pcw.calculateTargetInfo(
            15, 
            3.0, 
            -15.0, 
            0.271).getYaw());
        assertEquals(3.01, pcw.calculateTargetInfo(
            15, 
            3.0, 
            -15.0, 
            0.271).getDistance());
    }
}
