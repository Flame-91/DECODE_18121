package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightSubsystem {
    private Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        // Initialize your Limelight hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    // Call this to get the latest Limelight results
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    // Example: get AprilTag ID
    public int getAprilTagID() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getId();   // This gives AprilTag ID
        }
        return -1; // No tag detected
    }

    // Example: get robot pose (if AprilTag detected)
    public double[] getBotPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getBotpose() != null) {
            return result.getBotpose();  // returns [x,y,z,roll,pitch,yaw]
        }
        return null; // No pose available
    }
}
