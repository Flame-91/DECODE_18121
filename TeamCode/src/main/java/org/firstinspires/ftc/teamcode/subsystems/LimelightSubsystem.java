package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubsystem {
    private Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        // Initialize your Limelight hardware
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
    }

    // Get latest vision result
    private LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public int getAprilTagID() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTargetID();   // FTC API uses getTargetID()
        }
        return -1; // No tag detected
    }

    public Pose3D getBotPose() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getBotpose() != null) {
            return result.getBotpose();   // FTC API has getBotpose()
        }
        return null;
    }

    public boolean hasTarget() {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }

    public double getXOffset() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0.0;
    }

    public double getYOffset() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy();
        }
        return 0.0;
    }
}
