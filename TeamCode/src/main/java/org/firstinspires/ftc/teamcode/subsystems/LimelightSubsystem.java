package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    private final double distanceFromFloorToTagMeters = .7051;
    private final double limelightLensHeightMeters = 0.254;
    private final double limelightMountAngleDegrees = 15;
    private final double goalHeightMeters = 0.7493;
    double limelightHorizontalOffsetMeters = 15; // offset left or right from center of lens to center of robot in meters
    private final IMU imu;


    LLResult result;
    public LimelightSubsystem(HardwareMap hardwareMap, IMU imu, Telemetry telemetry, TelemetryPacket telemetryPacket, FtcDashboard dashboard) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        result = limelight.getLatestResult();

        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;
        this.imu = imu;

        dashboard.startCameraStream(limelight, 0);

        register();
    }

    @Override
    public void periodic() {
        result = limelight.getLatestResult();
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());

        telemetry.addData("hasTarget", hasTarget());
        telemetry.addData("getAprilTagID", getAprilTagID());
        telemetry.addData("pitchError", getPitchError());
        telemetry.addData("botPose", getBotPose());

        telemetryPacket.put("hasTarget", hasTarget());
        telemetryPacket.put("getAprilTagID", getAprilTagID());
        telemetryPacket.put("PitchError", getPitchError());
        telemetryPacket.put("botPose", getBotPose());
    }

    // Returns true if any target is visible
    public boolean hasTarget() {
        if (result != null) {
            return result.isValid();
        }
        return false;
    }

    // Returns the first AprilTag ID detected, or -1 if none
    public int getAprilTagID() {
        if (hasTarget()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                return fiducials.get(0).getFiducialId();
            }
        }
        return -1;
    }

    // Returns horizontal angle to target (yaw) in degrees, or -361 if no target
    public double getYawError() {
        if (hasTarget()) {
            return result.getTx();
        }
        return -361.0;
    }

    // Returns vertical angle to target (pitch) in degrees, or -361 if no target
    public double getPitchError() {
        if (hasTarget()) {
            return result.getTy();
        }
        return -361.0;
    }

    public double getPitchError(double offsetMeters) {
        if (hasTarget()) {
            double horizontalDistanceMeters = getHorizontalDistanceMeters();
            return Math.atan(((distanceFromFloorToTagMeters - limelightLensHeightMeters) + offsetMeters) / horizontalDistanceMeters);
        }
        return -361.0;
    }

    public double getHorizontalDistanceMeters() {
        if (hasTarget()) {
            double angleToGoalDegrees = limelightMountAngleDegrees + getPitchError();
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            return (goalHeightMeters - limelightLensHeightMeters) / Math.tan(angleToGoalRadians);
        }
        return -1;
    }

    private boolean isObelisk() {
         return getAprilTagID() == 21 || getAprilTagID() == 22 || getAprilTagID() == 23;
    }
    // returns the robot's center's position on the field if limelight can see an april tag

    public double[] getBotPose() {
        if (hasTarget() && !isObelisk()) {
            Pose3D botPose = result.getBotpose_MT2();
            return new double[]{botPose.getPosition().x, botPose.getPosition().y, botPose.getPosition().z, botPose.getOrientation().getRoll(), botPose.getOrientation().getPitch(), botPose.getOrientation().getYaw()};  // returns [x,y,z,roll,pitch,yaw] so getBotPose()[4] is pitch
        }
        return new double[]{};
    }

    // returns robot's center's position on field if ll can see april tag in Pose3D instead of double[] and returns null if LL can't see april tag
    public Pose3D getBotPosePose3D() {
        if (hasTarget() && !isObelisk()) {
            return result.getBotpose_MT2();
        }
        return null;
    }

    // Returns the motif pattern based on AprilTag ID
    public String[] motif() {
        int tagID = getAprilTagID();
        if (isObelisk()) {
            switch (tagID) {
                case 21: return new String[]{"g", "p", "p"};
                case 22: return new String[]{"p", "g", "p"};
                case 23: return new String[]{"p", "p", "g"};
            }
        }

        return new String[]{};
    }
}
