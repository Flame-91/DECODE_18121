package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightSubsystem {
    private Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        // Initialize your Limelight hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    // Get latest vision result
    private LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public int getAprilTagID() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                return id;
                // Use the ID as needed
            }   // FTC API uses getTargetID()

        }
        return -1; // No tag detected
    }

    public Pose3D getBotPose() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getBotpose() != null) {
            return result.getBotpose();  // returns [x,y,z,roll,pitch,yaw] so getBotPose()[4] is pitch
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
    public double getPitch() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTy(); // How far up or down the target is (degrees)
        }
        return -1;
    }

    public double getYaw() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx(); // How far up or down the target is (degrees)
        }
        return -1;
    }
}
