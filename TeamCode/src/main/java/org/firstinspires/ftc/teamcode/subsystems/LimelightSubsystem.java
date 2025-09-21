package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    private LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    // Returns true if any target is visible
    public boolean hasTarget() {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }

    // Returns the first AprilTag ID detected, or -1 if none
    public int getAprilTagID() {
        LLResult result = getLatestResult();
        if (hasTarget()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                return fiducials.get(0).getFiducialId();
            }
        }
        return -1;
    }

    // Returns horizontal angle to target (yaw) in degrees, or -361 if no target
    public double getYaw() {
        LLResult result = getLatestResult();
        if (hasTarget()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult tag = fiducials.get(0);
                    return tag.getTargetPoseRobotSpace().getOrientation().getYaw();
                }
            }
        }
        return -361.0;
    }

    // Returns vertical angle to target (pitch) in degrees, or -361 if no target
    public double getPitch() {
        LLResult result = getLatestResult();
        if (hasTarget()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                double x = tag.getTargetPoseRobotSpace().getPosition().x; // forward
                double z = tag.getTargetPoseRobotSpace().getPosition().z; // up/down
                return Math.toDegrees(Math.atan2(z, x));
            }
        }
        return -361.0;
    }

    // returns the robot's center's position on the field if limelight can see an april tag

    public double[] getBotPose() {
        LLResult result = getLatestResult();
        if (hasTarget()) {
            Pose3D botPose = result.getBotpose();
            return new double[]{botPose.getPosition().x, botPose.getPosition().y, botPose.getPosition().z, botPose.getOrientation().getRoll(), botPose.getOrientation().getPitch(), botPose.getOrientation().getYaw()};  // returns [x,y,z,roll,pitch,yaw] so getBotPose()[4] is pitch
        }
        return new double[]{};
    }

    // returns robot's center's position on field if ll can see april tag in Pose3D instead of double[] and returns null if LL can't see april tag
    public Pose3D getBotPosePose3D() {
        LLResult result = getLatestResult();
        if (hasTarget()) {
            return result.getBotpose();
        }
        return null;
    }

    // Returns the motif pattern based on AprilTag ID
    public String[] motif() {
        int tagID = getAprilTagID();
        if (tagID == 21) {
            return new String[]{"g", "p", "p"};
        } else if (tagID == 22) {
            return new String[]{"p", "g", "p"};
        } else if (tagID == 23) {
            return new String[]{"p", "p", "g"};
        }

        return new String[]{};
    }
}
