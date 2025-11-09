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

@Autonomous(name = "TestAuton")
public class TestAuton extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56.000, 8.000, 90);
    private final Pose finishPose = new Pose(71.5, 71.5, 90);

    private Path testPath;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        switch (pathState) {
            case 0:
                follower.followPath(testPath);
                telemetry.addData("X:", follower.getPose().getX());
                telemetry.addData("Y:", follower.getPose().getY());
                setPathState(1);
                break;
            case 1:
                telemetry.addData("X:", follower.getPose().getX());
                telemetry.addData("Y:", follower.getPose().getY());
                if (!follower.isBusy()) {
                    // done
                }
        }
    }

    public void buildPaths() {
        testPath = new Path(new BezierLine(startPose, finishPose));
        testPath.setLinearHeadingInterpolation(startPose.getHeading(), finishPose.getHeading());
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
