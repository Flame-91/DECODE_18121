package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightSubsystem {
    private static Limelight3A limelight;

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

    public double[] distanceFromTag() {
        LLResult result = limelight.getLatestResult();

        if (hasTarget()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0); // Get the first detected AprilTag
                double distanceX = tag.getTargetPoseRobotSpace().getPosition().y; // this might be wrong idk lol
                double distanceY = tag.getTargetPoseRobotSpace().getPosition().x; // swapping x and y because x is forward and backward and y is side to side for some reason
                double distanceZ = tag.getTargetPoseRobotSpace().getPosition().z;

                return new double[]{distanceX, distanceY, distanceZ};
                // Use distanceX, distanceY, and distanceZ as needed
//                telemetry.addData("Distance X", distanceX);
//                telemetry.addData("Distance Y", distanceY);
//                telemetry.addData("Distance Z", distanceZ);
            }
        }

        return new double[]{};
    }

    public double[] getBotPose() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid() && result.getBotpose() != null) {
            Pose3D botPose = result.getBotpose();
            return new double[]{botPose.getPosition().x, botPose.getPosition().y, botPose.getPosition().z, botPose.getOrientation().getRoll(), botPose.getOrientation().getPitch(), botPose.getOrientation().getYaw()};  // returns [x,y,z,roll,pitch,yaw] so getBotPose()[4] is pitch
        }
        return null;
    }

    public double xYDistanceFromTag() {
        if (hasTarget()) {
            return Math.sqrt(Math.pow(distanceFromTag()[0], 2) + Math.pow(distanceFromTag()[1], 2)); // pythag to find xy distance
        }
        return 0;
    }

    public double[] autoAlign() {
        if (hasTarget()) {
            double buffer = 0.03;
            double xCorrection = 0;
            double yCorrection = 0;
            double yawCorrection;
            if (distanceFromTag()[0] > buffer) {
                xCorrection = -0.2;
            } else if (distanceFromTag()[0] < -buffer) {
                xCorrection = 0.2;
            }

            if (distanceFromTag()[1] > buffer) {
                yCorrection = -0.2;
            } else if (distanceFromTag()[1] < -buffer) {
                yCorrection = 0.2;
            }

            if (getYaw() > 0) {
                yawCorrection = -0.15;
            } else {
                yawCorrection = 0.15;
            }

            return new double[]{xCorrection, yCorrection, yawCorrection};
        }

        return new double[]{};
    }

    public boolean hasTarget() {
        LLResult result = getLatestResult();
        return result != null && result.isValid();
    }

//    public double getXOffset() {
//        LLResult result = getLatestResult();
//        if (hasTarget()) {
//            return result.getTx();
//        }
//        return 0.0;
//    }
//
//    public double getYOffset() {
//        LLResult result = getLatestResult();
//        if (hasTarget()) {
//            return result.getTy();
//        }
//        return 0.0;
//    }
    public double getPitch() {
        LLResult result = getLatestResult();
        if (hasTarget()) {
            return result.getTy(); // How far up or down the target is (degrees)
        }
        return -361.0;
    }

    public double getYaw() {
        LLResult result = getLatestResult();
        if (hasTarget()) {
            return result.getTx(); // How far left or right the target is (degrees)
        }
        return -361.0;
    }

    public String[] motif() {
        int aprilTag = getAprilTagID();
        if (aprilTag == 21) {
            return new String[]{"g", "p", "p"};
        } else if (aprilTag == 22) {
            return new String[]{"p", "g", "p"};
        } else if (aprilTag == 23) {
            return new String[]{"p", "p", "g"};
        }

        return new String[]{};
    }
}
