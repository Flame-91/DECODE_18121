package org.firstinspires.ftc.teamcode.game;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

@Autonomous(name = "Auton")
@Configurable
public class Auton extends OpMode {
    public static double startPoseX = 56;
    public static double startPoseY = 8;
    public static double startPoseHeading = 90;
    public static double scorePoseX = 115;
    public static double scorePoseY = 145;
    public static double scorePoseHeading = 90;
    public static double reloadPoseX = 15;
    public static double reloadPoseY = 20;
    public static double reloadHeading = 45;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(startPoseX, startPoseY, startPoseHeading);
    private final Pose scorePose = new Pose(scorePoseX, scorePoseY, scorePoseHeading);
    private final Pose reloadPose = new Pose(reloadPoseX, reloadPoseY, reloadHeading);
    private boolean score;
    private Path scorePreload;
    private Path reload;
    private Path scoreReload;
    private Paths paths;
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
    private void EndConstraints(Path p) {
        p.setTranslationalConstraint(1.5);         // inches
        p.setHeadingConstraint(Math.toRadians(5));  // radians
        p.setVelocityConstraint(2);                 // inches/sec
        p.setTValueConstraint(0.98);                // percent of path
        p.setTimeoutConstraint(300);                // ms
    }

    public void buildPaths() {
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
        // 1) Run flywheel until time hits "runTime"
        if (elapsedTime < runTime) {
            flyWheelSubsystem.runFlywheel(0.567);
            return false;
        }
        // 2) After flywheel is at speed, run servos for 0.25s
        if (elapsedTime < runTime + 0.25) {
            flyWheelSubsystem.runFlywheelServos(1);
            return false;
        }
        // 3) Scoring complete
        flyWheelSubsystem.runFlywheelServos(0);
        return true;
    }

    public void stopFlywheel() {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
//                telemetry.addData("");
                // goes to goal to score preloads
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

    public static class Paths {
        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(startPoseX, startPoseY), new Pose(scorePoseX, scorePoseY)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(220))
                    .build();
        }
    }
}
