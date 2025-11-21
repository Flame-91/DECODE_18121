package org.firstinspires.ftc.teamcode.game;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.util.Paths;

import static org.firstinspires.ftc.teamcode.commands.FlywheelCommand.flywheelServoBreaktime1;
import static org.firstinspires.ftc.teamcode.commands.FlywheelCommand.flywheelServoBreaktime2;
import static org.firstinspires.ftc.teamcode.commands.FlywheelCommand.flywheelServoRuntime;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

@Autonomous(name="CompAutoBlue")
public class CompAutoBlue extends OpMode {
    private AutoState currentState = AutoState.GO_TO_PRELOAD_SCORE;
    private ScoreState currentScoreState = ScoreState.RUN1;

    FlyWheelSubsystem flyWheelSubsystem;
    Paths paths;
    ElapsedTime servoElapsedTime;
    Timer reloadTime;

    private int shotCount = 0;
    private boolean pathStarted = false;

    // --- Enums for State Machines ---
    private enum AutoState {
        GO_TO_PRELOAD_SCORE,
        DRIVE_TO_RELOAD,
        DRIVE_TO_RELOAD1,
        DRIVE_TO_RELOAD2,
        DRIVE_TO_SCORE,
        SCORE_1,
        IDLE
    }

    private enum ScoreState {
        RUN1,
        BREAK1,
        RUN2,
    }

    public void init() {
        servoElapsedTime = new ElapsedTime();
        reloadTime = new Timer();

        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Initialized");
    }

    public void loop() {
        flyWheelSubsystem.runFlywheel(0.57067);

        follower.update();

        autoStateUpdate();

        // Telemetry updates for debugging
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("current AutoState: ", currentState);
        telemetry.addData("current ScoreState: ", currentScoreState);
        telemetry.update();
    }

    private void autoStateUpdate() {
        switch (currentState) {
            case GO_TO_PRELOAD_SCORE:
                if (!pathStarted) {
                    follower.followPath(paths.preload_score_blue);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    servoElapsedTime.reset();
                    currentState = AutoState.SCORE_1;
                }
                break;

            case SCORE_1:
                if (scoreAuto()) {
                    currentState = AutoState.DRIVE_TO_RELOAD;
                }
                break;

            case DRIVE_TO_RELOAD:
                if (!pathStarted) {
                    follower.followPath(paths.to_reload_blue);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    reloadTime.resetTimer();
                    currentState = AutoState.DRIVE_TO_RELOAD1;
                }
                break;

            case DRIVE_TO_RELOAD1:
                if (!pathStarted) {
                    follower.followPath(paths.to_reload_blue_1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    reloadTime.resetTimer();
                    currentState = AutoState.DRIVE_TO_RELOAD2;
                }

            case DRIVE_TO_RELOAD2:
                if (!pathStarted) {
                    follower.followPath(paths.to_reload_blue_2);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    reloadTime.resetTimer();
                    currentState = AutoState.DRIVE_TO_SCORE;
                }

            case DRIVE_TO_SCORE:
                if (!pathStarted && reloadTime.getElapsedTimeSeconds() > 2) {
                    follower.followPath(paths.to_score_blue);
                    pathStarted = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    pathStarted = false;
                    currentState = AutoState.SCORE_1;
                }
                break;

            case IDLE:
                flyWheelSubsystem.runFlywheel(0); // Stop the flywheel
                break;
        }
    }

    private boolean scoreAuto() {
        switch (currentScoreState) {
            case RUN1:
                flyWheelSubsystem.runFlywheelServos(1);
                if (servoElapsedTime.seconds() >= flywheelServoRuntime) {
                    servoElapsedTime.reset();
                    currentScoreState = ScoreState.BREAK1;
                }
                return false;
            case BREAK1:
                flyWheelSubsystem.runFlywheelServos(0);
                if (servoElapsedTime.seconds() >= flywheelServoBreaktime1) {
                    servoElapsedTime.reset();
                    currentScoreState = ScoreState.RUN2;
                }
                return false;
            case RUN2:
                flyWheelSubsystem.runFlywheelServos(1);
                if (servoElapsedTime.seconds() >= flywheelServoRuntime) {
                    servoElapsedTime.reset();
                    return true;
                }
                return false;
        }
        return false;
    }

    @Override
    public void stop() {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }
}