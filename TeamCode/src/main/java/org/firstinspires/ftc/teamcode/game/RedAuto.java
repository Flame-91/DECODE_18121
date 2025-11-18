package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKD;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKI;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.util.GlobalConstants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedAuto")
public class RedAuto extends OpMode {
    private IMU imu;
    private Follower follower;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket telemetryPacket = new TelemetryPacket();
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(56, 8, 90);
    private final Pose scorePose = new Pose(70, 14, 63);
    private final Pose intakePoseOne = new Pose(34.678899082568805, 35.5045871559633, 180);
    private final Pose intakePoseTwo = new Pose(24.605504587155963, 35.6697247706422, 180);
    private boolean score;
    private Path scorePreload;
    private Path intakeOne;
    private Path intakeTwo;
    private Path scoreReload;
    private long lastTime;
    FlywheelSubsystem flyWheelSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limelightSubsystem;
    PivotSubsystem pivotSubsystem;
    PIDController pivotPIDController;
    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        flyWheelSubsystem = new FlywheelSubsystem(hardwareMap, telemetry, telemetryPacket);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, telemetryPacket);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, imu, telemetry, telemetryPacket, dashboard);
        pivotSubsystem = new PivotSubsystem(hardwareMap, telemetry, telemetryPacket);

        pivotPIDController = new PIDController(pivotKP, pivotKI, pivotKD, 0.5);
    }

    @Override
    public void loop() {
        dashboard.sendTelemetryPacket(telemetryPacket);
        telemetry.update();
        updateLimelight();
        executePivot();
        follower.update();
        autonomousPathUpdate();
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        intakeOne = new Path(new BezierLine(scorePose, intakePoseOne));
        intakeOne.setLinearHeadingInterpolation(scorePose.getHeading(), intakePoseOne.getHeading());

        intakeTwo = new Path(new BezierLine(intakePoseOne, intakePoseTwo));
        intakeTwo.setLinearHeadingInterpolation(intakePoseOne.getHeading(), intakePoseTwo.getHeading());

        scoreReload = new Path(new BezierLine(intakePoseTwo, scorePose));
        scoreReload.setLinearHeadingInterpolation(intakePoseTwo.getHeading(), scorePose.getHeading());
    }

    public boolean score(double elapsedTime, double runTime) { // elapsed time is how much time it has been since score was first called, and run time is how uch time it should run the motor
        if (runTime < elapsedTime) {
            flyWheelSubsystem.setFlywheelMotorPower(1); // runs motor as long as elapsed time is less than specified runtime
        } else if (runTime + 0.25 < elapsedTime) {
            flyWheelSubsystem.setFlywheelServoPower(1); // runs servos for 0.25 secs after motor runs
        } else {
            flyWheelSubsystem.setFlywheelServoPower(0); // stops running servos but keeps motor running until stopFlywheel in psm
            return true;
        }
        return false;
    }

    public void executePivot() {
        if (limelightSubsystem.hasTarget()) {
            double pivotPositionAngle = pivotSubsystem.convertPivotTicksToAngle(pivotSubsystem.getCurrentPivotPosition());
            double pitchError = limelightSubsystem.getPitchError(0.42545) - pivotPositionAngle; // 0.42545 is how far up from the center of the april tag we need to shoot

            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;
            lastTime = currentTime;
            double output = pivotPIDController.calculate(pitchError, deltaTime);

            pivotSubsystem.setPivotTargetPosition(pivotSubsystem.convertPivotAngleToTicks(pitchError));
            pivotSubsystem.setPivotPower(-output); // opposite so pivot corrects in right direction
        }
    }

    public void updateLimelight() {
        if (!limelightSubsystem.hasTarget()) return;
        Pose3D currentPosition = limelightSubsystem.getBotPosePose3D(); // gets position of robot using limelight MT2 (very very accurate)
        if (currentPosition == null) return; // current position is null if its looking at an obelisk (since obelisk is not meant for precision localization)
        double xInches = (currentPosition.getPosition().x * 39.3701) + 72; // converts meters to inches and adding 72 for pedro coordinates
        double yInches = (currentPosition.getPosition().y * 39.3701) + 72;
        double heading = Math.toRadians(currentPosition.getOrientation().getYaw()); // outputs in degrees, so converting to radians for pedro
        Pose currentPose = new Pose(xInches, yInches, heading);
        follower.setPose(currentPose);
    }

    public void stopFlywheel() {
        flyWheelSubsystem.setFlywheelServoPower(0);
        flyWheelSubsystem.setFlywheelMotorPower(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload); // goes to goal to score preloads
                if (!follower.isBusy()) {
                    actionTimer.resetTimer(); // resets timer for next case
                    setPathState(1);
                }
                break;
            case 1:
                score = score(actionTimer.getElapsedTime(), 3.25); // scores first artifact
                if (score) {
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                score = score(actionTimer.getElapsedTime(), 0.5); // scores second artifact (less runtime since motor is already spinning but we still want a little break so artifacts dont hit each other mid air)
                if (score) {
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                score = score(actionTimer.getElapsedTime(), 0.5); // scores last artifact
                if (score) {
                    stopFlywheel(); // stops flywheels
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(intakeOne); // goes to artifact to reload
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (actionTimer.getElapsedTime() > 0.5) {  // gives time for intake to intake
                    setPathState(6);
                }
                break;
            case 6:
                follower.followPath(scoreReload); // goes to score area
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                score = score(actionTimer.getElapsedTime(), 3); // scores first artifact
                if (score) {
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                score = score(actionTimer.getElapsedTime(), 0.5); // scores second artifact
                if (score) {
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                score = score(actionTimer.getElapsedTime(), 0.5); // scores last artifact
                if (score) {
                    stopFlywheel(); // stops flywheels
                    actionTimer.resetTimer();
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        GlobalConstants.imuOffset = imu.getRobotYawPitchRollAngles().getYaw();
    }
}


