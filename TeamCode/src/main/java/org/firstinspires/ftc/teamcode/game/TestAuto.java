package org.firstinspires.ftc.teamcode.game;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

@Autonomous(name = "nandnaAuton")
public class TestAuto extends OpMode {

    private Follower follower;

    private Timer stateTimer;     // resets on each state transition

    private State state;

    private final Pose startPose = new Pose(56, 8, 90);
    private final Pose scorePose = new Pose(115, 125, 215);
    private final Pose reloadPose = new Pose(15, 20, 45);

    private Path scorePreload;
    private Path reload;
    private Path scoreReload;

    FlyWheelSubsystem flyWheelSubsystem;

    private enum State {
        DRIVE_PRELOAD,
        SCORE1,
        SCORE2,
        SCORE3,
        DRIVE_RELOAD,
        WAIT_RELOAD,
        DRIVE_SCORE_RELOAD,
        SCORE4,
        SCORE5,
        SCORE6,
        IDLE
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);

        stateTimer = new Timer();

        buildPaths();
        follower.setStartingPose(startPose);

        switchState(State.DRIVE_PRELOAD);
    }

    @Override
    public void loop() {
        follower.update();
        updateState();
    }

    // Switch state and reset timer
    private void switchState(State newState) {
        state = newState;
        stateTimer.resetTimer();
    }

    private double stateTime() {
        return stateTimer.getElapsedTime();
    }

    private void EndConstraints(Path p) {
        p.setTranslationalConstraint(1.5);
        p.setHeadingConstraint(Math.toRadians(5));
        p.setVelocityConstraint(2);
        p.setTValueConstraint(0.98);
        p.setTimeoutConstraint(300);
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        EndConstraints(scorePreload);

        reload = new Path(new BezierLine(scorePose, reloadPose));
        reload.setLinearHeadingInterpolation(scorePose.getHeading(), reloadPose.getHeading());
        EndConstraints(reload);

        scoreReload = new Path(new BezierLine(reloadPose, scorePose));
        scoreReload.setLinearHeadingInterpolation(reloadPose.getHeading(), scorePose.getHeading());
        EndConstraints(scoreReload);
    }

    public boolean score(double elapsedTime, double runTime) {
        if (elapsedTime < runTime) {
            flyWheelSubsystem.runFlywheel(0.567);
            return false;
        }

        if (elapsedTime < runTime + 0.25) {
            flyWheelSubsystem.runFlywheelServos(1);
            return false;
        }

        flyWheelSubsystem.runFlywheelServos(0);
        return true;
    }

    public void stopFlywheel() {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }

    private void updateState() {

        switch (state) {

            case DRIVE_PRELOAD:
                if (stateTime() == 0)
                    follower.followPath(scorePreload);

                if (!follower.isBusy())
                    switchState(State.SCORE1);
                break;

            case SCORE1:
                if (score(stateTime(), 3))
                    switchState(State.SCORE2);
                break;

            case SCORE2:
                if (score(stateTime(), 0.5))
                    switchState(State.SCORE3);
                break;

            case SCORE3:
                if (score(stateTime(), 0.5)) {
                    stopFlywheel();
                    switchState(State.DRIVE_RELOAD);
                }
                break;

            case DRIVE_RELOAD:
                if (stateTime() == 0)
                    follower.followPath(reload);

                if (!follower.isBusy())
                    switchState(State.WAIT_RELOAD);
                break;

            case WAIT_RELOAD:
                if (!follower.isBusy()) {
                    // must wait 3 SECONDS AFTER arrival
                    if (stateTime() > 3)
                        switchState(State.DRIVE_SCORE_RELOAD);
                } else {
                    // Still moving — restart wait timer
                    stateTimer.resetTimer();
                }
                break;

            case DRIVE_SCORE_RELOAD:
                if (stateTime() == 0)
                    follower.followPath(scoreReload);

                if (!follower.isBusy())
                    switchState(State.SCORE4);
                break;

            case SCORE4:
                if (score(stateTime(), 3))
                    switchState(State.SCORE5);
                break;

            case SCORE5:
                if (score(stateTime(), 0.5))
                    switchState(State.SCORE6);
                break;

            case SCORE6:
                if (score(stateTime(), 0.5)) {
                    stopFlywheel();
                    switchState(State.IDLE);
                }
                break;

            case IDLE:
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("State Time", stateTime());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }
}
