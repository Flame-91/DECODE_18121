package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKD;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKF;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKI;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Paths.BluePaths;

@Autonomous(name = "BlueAuto")
@Configurable
@Config
public class BlueAuto extends OpMode {
    public static double servoRuntime = 0.4;
    public static double servoBreaktime1 = 0.75;
    public static double servoBreaktime2 = 1.5;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket telemetryPacket = new TelemetryPacket();

    private boolean pathStarted = false;

    private enum AutoState {
        GO_TO_SCORE_1,
        SCORE_1,
        GO_TO_RELOAD_1_1,
        GO_TO_RELOAD_1_2,
        GO_TO_SCORE_2,
        SCORE_2,
        GO_TO_RELOAD_2_1,
        GO_TO_RELOAD_2_2,
        GO_TO_SCORE_3,
        SCORE_3,
        GATE,
        GATE_WAIT,
        GO_TO_RELOAD_3_1,
        GO_TO_RELOAD_3_2,
        GO_TO_SCORE_4,
        SCORE_4,
        DONE
    }

    private enum ScoreState {
        RUN_1,
        BREAK_1,
        RUN_2,
        BREAK_2,
        RUN_3
    }

    private AutoState autoState;
    private ScoreState scoreState;
    FlywheelSubsystem flywheelSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limelightSubsystem;
    PivotSubsystem pivotSubsystem;
    double totalPivotOutput;
    private IMU imu;
    ElapsedTime servoElapsedTime;
    BluePaths paths;
    Follower follower;
    private long lastTime;
    PIDController pivotPIDController;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        autoState = AutoState.GO_TO_SCORE_1;
        scoreState = ScoreState.RUN_1;
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap, telemetry, telemetryPacket);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, telemetryPacket);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, imu, telemetry, telemetryPacket, dashboard);

        servoElapsedTime = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        paths = new BluePaths(follower);
        lastTime = System.nanoTime();
        pivotPIDController = new PIDController(pivotKP, pivotKI, pivotKD, 0.75);
        totalPivotOutput = 0;

        telemetry.addData("Status", "Initialized");
        telemetryPacket.put("Status", "Initialized");
        telemetry.update();
        dashboard.sendTelemetryPacket(telemetryPacket);
    }

    @Override
    public void loop() {
        autoStateUpdate();
        pivotUpdate();
        limelightUpdate();
        intakeUpdate();
        flywheelMotorUpdate();
        follower.update();

        telemetry.addData("Current autoState", autoState);
        telemetry.addData("Current scoreState", scoreState);

        telemetryPacket.put("Current autoState", autoState);
        telemetryPacket.put("Current scoreState", scoreState);

        telemetry.update();
        dashboard.sendTelemetryPacket(telemetryPacket);
    }

    public void autoStateUpdate() {
        switch (autoState) {
            case GO_TO_SCORE_1:
                if (!pathStarted) {
                    follower.followPath(paths.score_1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    servoElapsedTime.reset();
                    autoState = AutoState.SCORE_1;
                }
                break;

            case SCORE_1:
                if (scoreStateUpdate()) { autoState = AutoState.GO_TO_RELOAD_1_1; }
                break;

            case GO_TO_RELOAD_1_1:
                if (!pathStarted) {
                    follower.followPath(paths.reload_1_1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoState = AutoState.GO_TO_RELOAD_1_2;
                }
                break;

            case GO_TO_RELOAD_1_2:
                if (!pathStarted) {
                    follower.followPath(paths.reload_1_2);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoState = AutoState.GO_TO_SCORE_2;
                }
                break;

            case GO_TO_SCORE_2:
                if (!pathStarted) {
                    follower.followPath(paths.score_2) ;
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    servoElapsedTime.reset();
                    autoState = AutoState.SCORE_2;
                }
                break;

            case SCORE_2:
                if (scoreStateUpdate()) { autoState = AutoState.GO_TO_RELOAD_2_1; }
                break;

            case GO_TO_RELOAD_2_1:
                if (!pathStarted) {
                    follower.followPath(paths.reload_2_1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoState = AutoState.GO_TO_RELOAD_2_2;
                }
                break;

            case GO_TO_RELOAD_2_2:
                if (!pathStarted) {
                    follower.followPath(paths.reload_2_2);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoState = AutoState.GO_TO_SCORE_3;
                }
                break;

            case GO_TO_SCORE_3:
                if (!pathStarted) {
                    follower.followPath(paths.score_3);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    servoElapsedTime.reset();
                    autoState = AutoState.SCORE_3;
                }
                break;

            case SCORE_3:
                if (scoreStateUpdate()) { autoState = AutoState.GATE; }
                break;

            case GATE:
                if (!pathStarted) {
                    follower.followPath(paths.gate);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    servoElapsedTime.reset();
                    autoState = AutoState.GATE_WAIT;
                }
                break;

            case GATE_WAIT:
                if (servoElapsedTime.seconds() >= 0.5) {
                    autoState = AutoState.GO_TO_RELOAD_3_1;
                }
                break;

            case GO_TO_RELOAD_3_1:
                if (!pathStarted) {
                    follower.followPath(paths.reload_3_1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoState = AutoState.GO_TO_RELOAD_3_2;
                }
                break;

            case GO_TO_RELOAD_3_2:
                if (!pathStarted) {
                    follower.followPath(paths.reload_3_2);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    autoState = AutoState.GO_TO_SCORE_4;
                }
                break;

            case GO_TO_SCORE_4:
                if (!pathStarted) {
                    follower.followPath(paths.score_4);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    servoElapsedTime.reset();
                    autoState = AutoState.SCORE_4;
                }
                break;

            case SCORE_4:
                if (scoreStateUpdate()) { autoState = AutoState.DONE; }
                break;

            case DONE:
                requestOpModeStop();
                break;
        }
    }

    public boolean scoreStateUpdate() {
        switch (scoreState) {
            case RUN_1:
                flywheelSubsystem.setFlywheelServoPower(1);
                if (servoElapsedTime.seconds() >= servoRuntime) {
                    servoElapsedTime.reset();
                    scoreState = ScoreState.BREAK_1;
                }
                return false;
            case BREAK_1:
                flywheelSubsystem.setFlywheelServoPower(0);
                if (servoElapsedTime.seconds() >= servoBreaktime1) {
                    servoElapsedTime.reset();
                    scoreState = ScoreState.RUN_2;
                }
                return false;
            case RUN_2:
                flywheelSubsystem.setFlywheelServoPower(1);
                if (servoElapsedTime.seconds() >= servoRuntime) {
                    servoElapsedTime.reset();
                    scoreState = ScoreState.BREAK_2;
                }
                return false;
            case BREAK_2:
                flywheelSubsystem.setFlywheelServoPower(0);
                if (servoElapsedTime.seconds() >= servoBreaktime2) {
                    servoElapsedTime.reset();
                    scoreState = ScoreState.RUN_3;
                }
                return false;
            case RUN_3:
                flywheelSubsystem.setFlywheelServoPower(1);
                if (servoElapsedTime.seconds() >= servoRuntime) {
                    servoElapsedTime.reset();
                    scoreState = ScoreState.RUN_1;
                    return true;
                }
                return false;
        }
        return false;
    }

    public void pivotUpdate() {
        if (limelightSubsystem.hasTarget()) {
            double pivotPositionAngle = pivotSubsystem.convertPivotTicksToAngle(pivotSubsystem.getCurrentPivotPosition());
            double targetPosition = limelightSubsystem.getPitchError(0.42545);
            double pitchError = targetPosition - pivotPositionAngle; // 0.42545 is how far up from the center of the april tag we need to shoot

            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;
            lastTime = currentTime;
            double output = pivotPIDController.calculate(pitchError, deltaTime);
            double feedForward = pivotKF * Math.cos(Math.toRadians(targetPosition));

            totalPivotOutput = output + feedForward;
            pivotSubsystem.setPivotPower(totalPivotOutput);
        }
    }

    public void limelightUpdate() {
        if (!limelightSubsystem.hasTarget()) return;
        Pose3D currentPosition = limelightSubsystem.getBotPosePose3D(); // gets position of robot using limelight MT2 (very very accurate)
        if (currentPosition == null) return; // current position is null if its looking at an obelisk (since obelisk is not meant for precision localization)
        double xInches = (currentPosition.getPosition().x * 39.3701) + 72; // converts meters to inches and adding 72 for pedro coordinates
        double yInches = (currentPosition.getPosition().y * 39.3701) + 72;
        double heading = Math.toRadians(currentPosition.getOrientation().getYaw()); // outputs in degrees, so converting to radians for pedro
        Pose currentPose = new Pose(xInches, yInches, heading);
        follower.setPose(currentPose);
    }

    public void intakeUpdate() {
        intakeSubsystem.setIntakeServoPower(1); // add more here to customize
    }

    public void flywheelMotorUpdate() {
        flywheelSubsystem.setFlywheelMotorPower(1); // add more here to customize
    }
}