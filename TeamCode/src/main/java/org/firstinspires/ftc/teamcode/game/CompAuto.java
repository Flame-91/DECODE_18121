package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.widget.Switch;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.util.Paths;

@Autonomous
public class CompAuto extends OpMode {
    private AutoState currentState = AutoState.GO_TO_PRELOAD_SCORE;
    FlyWheelSubsystem flyWheelSubsystem;
    Paths paths;
    ElapsedTime elapsedTime;
    Timer reloadTime;
    private int shotCount = 0;
    private boolean pathStarted = false; // Needed to make the paths run once (instead of being called every loop)


    private enum AutoState {
        GO_TO_PRELOAD_SCORE,
        DRIVE_TO_RELOAD,
        DRIVE_TO_SCORE,
        SCORE_1,
        SCORE_2,
        SCORE_3,
        IDLE
    }

    public void init() {
        elapsedTime = new ElapsedTime();
        reloadTime = new Timer();
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        paths = new Paths(follower);
    }

    public void loop() {
        flyWheelSubsystem.runFlywheel(0.57067);
        follower.update();
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getHeading());
        telemetry.addData("current state: ", currentState);
        telemetry.update();
        autoStateUpdate();
    }

    // ** Switch States **
    private void autoStateUpdate() {
        switch (currentState) {
            // -- GO TO PRELOAD SCORE --
            case GO_TO_PRELOAD_SCORE:
                if (!pathStarted) {
                    follower.followPath(paths.preload_score);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    currentState = AutoState.SCORE_1;
                }
                break;
            //  -- SCORE FIRST 3 RINGS --
            case SCORE_1:
                if (scoreAuto()) {
                    shotCount = 0;
                    currentState = AutoState.DRIVE_TO_RELOAD;
                }
                break;
            // -- DRIVE TO RELOAD --
            case DRIVE_TO_RELOAD:
                if (!pathStarted) {
                    follower.followPath(paths.to_reload);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    reloadTime.resetTimer();
                    currentState = AutoState.DRIVE_TO_SCORE;
                }
                break;
            // -- DRIVE TO SCORE --
            case DRIVE_TO_SCORE:
                if (!pathStarted && reloadTime.getElapsedTimeSeconds() > 2) {
                    follower.followPath(paths.to_score);
                    pathStarted = true;
                }

                if (pathStarted && !follower.isBusy()) {
                    pathStarted = false;
                    currentState = AutoState.SCORE_1;
                }
                break;
                // -- IDLE --
            case IDLE:
                break;
        }
    }

    // ** HELPER METHOD(S) **
    private boolean scoreAuto() {
        if (shotCount == 0) {
            flyWheelSubsystem.runFlywheelServos(1);
            elapsedTime.reset();
            shotCount++;
            return false;
        }

        if (elapsedTime.seconds() > 1.5) {
            flyWheelSubsystem.runFlywheelServos(1);
            elapsedTime.reset();
            shotCount++;

            if (shotCount >= 3) {
                return true;
            }
        }

        return false;
    }
}