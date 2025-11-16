package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.widget.Switch;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.util.Paths;

@Autonomous
public class CompAuto extends OpMode {
    private AutoState currentState = AutoState.GO_TO_PRELOAD_SCORE;
    FlyWheelSubsystem flyWheelSubsystem;
    Paths paths;
    Timer scoreTimer;
    private boolean firstShotDone = false;
    private int shotCount = 0;


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
        scoreTimer = new Timer();
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
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

    // ** STATE MACHINE **
    private void autoStateUpdate() {
        switch (currentState) {
            case GO_TO_PRELOAD_SCORE:
                follower.followPath(paths.preload_score);
                if (!follower.isBusy()) {
                    currentState = AutoState.SCORE_1;
                }
            case SCORE_1:
                if (scoreAuto()) {
                    shotCount = 0;
                }
                if (!follower.isBusy()) {
                    currentState = AutoState.DRIVE_TO_RELOAD;
                }
                break;
            case DRIVE_TO_RELOAD:
                follower.followPath(paths.to_reload);
                if (!follower.isBusy()) {
                    currentState = AutoState.DRIVE_TO_SCORE;
                }
                break;
            case DRIVE_TO_SCORE:
                follower.followPath(paths.to_score);
                if (!follower.isBusy()) {
                    currentState = AutoState.DRIVE_TO_RELOAD;
                }
                break;
            case IDLE:
                break;
        }
    }

    // ** HELPER METHODS **
    private boolean scoreAuto() {
        if (shotCount == 0) {
            flyWheelSubsystem.runFlywheelServos(1);
            scoreTimer.resetTimer();
            shotCount++;
            return false;
        }

        if (scoreTimer.getElapsedTimeSeconds() > 1.5) {
            flyWheelSubsystem.runFlywheelServos(1);
            scoreTimer.resetTimer();
            shotCount++;

            if (shotCount >= 3) {
                return true;
            }
        }

        return false;
    }
}