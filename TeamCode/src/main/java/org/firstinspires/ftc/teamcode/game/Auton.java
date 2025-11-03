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
import com.seattlesolvers.solverslib.util.Timing;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auton")
public class Auton extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean score1;
    private boolean score2;
    private boolean score3;
    private final Pose startPose = new Pose(56, 8, 90);
    private final Pose scorePose = new Pose(115, 125, 215);
    private final Pose reloadPose = new Pose(15, 20, 45);
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

    public boolean score(long timerLength) {
        flyWheelSubsystem.runFlywheel(1);
        Timing.Timer timer = new Timing.Timer(timerLength, TimeUnit.SECONDS);
        if (timer.done()) {
            Timing.Timer newTimer = new Timing.Timer(250, TimeUnit.MILLISECONDS);
            flyWheelSubsystem.runFlywheelServos(.567);
            return newTimer.done();
        }
        return false;
    }

    public void stopFlywheelMotor() {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                score1 = score(3);
                if (!score1) {
                    break;
                }
                if (!follower.isBusy() && !score)
                    setPathState(2);
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(scoreReload);
                    if (!follower.isBusy()) setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    score = score();
                    if (!score) {
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(reload);
                    setPathState(-1);
                }
        }
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
