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

@Autonomous(name = "Auton")
public class Auton extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(56, 8, 90);
    private final Pose scorePose = new Pose(115, 125, 215);
    private final Pose reloadPose = new Pose(15, 20, 45);
    private boolean score;
    private Path scorePreload;
    private Path reload;
    private Path scoreReload;
    FlyWheelSubsystem flyWheelSubsystem;
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getHeading());
        telemetry.addData("current state", pathState);
        telemetry.update();
        autonomousPathUpdate();
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        reload = new Path(new BezierLine(scorePose, reloadPose));
        reload.setLinearHeadingInterpolation(scorePose.getHeading(), reloadPose.getHeading());

        scoreReload = new Path(new BezierLine(reloadPose, scorePose));
        scoreReload.setLinearHeadingInterpolation(reloadPose.getHeading(), scorePose.getHeading());
    }

    public boolean score(double elapsedTime, double runTime) { // elapsed time is how much time it has been since score was first called, and run time is how uch time it should run the motor
        if (runTime < elapsedTime) {
            flyWheelSubsystem.runFlywheel(0.567); // runs motor as long as elapsed time is less than specified runtime
        } else if (runTime + 0.25 < elapsedTime) {
            flyWheelSubsystem.runFlywheelServos(1); // runs servos for 0.25 secs after motor runs
        } else {
            flyWheelSubsystem.runFlywheelServos(0); // stops running servos but keeps motor running until stopFlywheel in psm
            return true;
        }
        return false;
    }

    public void stopFlywheel() {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
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
                score = score(actionTimer.getElapsedTime(), 3); // scores first artifact
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
                follower.followPath(reload); // goes to human player zone
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (actionTimer.getElapsedTime() > 3) { // gives human player time to put artifacts in
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
}
