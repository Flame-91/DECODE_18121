package org.firstinspires.ftc.teamcode.game;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.FlywheelCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

@Autonomous(name = "CompAutoBlueNew")
@Configurable
public class CompAutoBlueNew extends OpMode {
    public static double FLYWHEEL_POWER = 0.4867;
    public static double FLYWHEEL_WAIT_TIME = 5.0;
    public static double SCORE_RUN_TIME = FlywheelCommand.flywheelServoRuntime;
    public static double SCORE_BREAK_TIME = 3;

    private AutoState currentState = AutoState.MOVE;
    private Paths paths;

    private Follower follower;
    private ElapsedTime elapsedTime;
    private FlyWheelSubsystem flywheelSubsystem;

    private TelemetryManager panelsTelemetry;

    private boolean pathStarted = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23.968, 126.853, Math.toRadians(325)));
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        flywheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        paths = new Paths(follower);
    }

    @Override
    public void loop() {
        if (currentState != AutoState.MOVE) {
            flywheelSubsystem.runFlywheel(FLYWHEEL_POWER);
        }
        switch (currentState) {
            case REV_FLYWHEEL:
                flywheelSubsystem.runFlywheelServos(0);

                if (elapsedTime.seconds() >= FLYWHEEL_WAIT_TIME) { // Wait for flywheel
                    elapsedTime.reset();
                    currentState = AutoState.RUN1;
                    panelsTelemetry.addData("Status", "RUN1");
                }
                break;
            case RUN1:
                flywheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= SCORE_RUN_TIME) {
                    elapsedTime.reset();
                    currentState = AutoState.BREAK1;
                    panelsTelemetry.addData("Status", "BREAK1");
                }
                break;

            case BREAK1:
                flywheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= SCORE_BREAK_TIME) {
                    elapsedTime.reset();
                    currentState = AutoState.RUN2;
                    panelsTelemetry.addData("Status", "RUN2");
                }
                break;

            case RUN2:
                flywheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= SCORE_RUN_TIME) {
                    elapsedTime.reset();
                    currentState = AutoState.BREAK2;
                    panelsTelemetry.addData("Status", "BREAK2");
                }
                break;

            case BREAK2:
                flywheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= SCORE_BREAK_TIME) {
                    elapsedTime.reset();
                    currentState = AutoState.RUN3;
                    panelsTelemetry.addData("Status", "RUN3");
                }
                break;

            case RUN3:
                flywheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= SCORE_RUN_TIME) {
                    elapsedTime.reset();
                    pathStarted = false;
                    flywheelSubsystem.runFlywheelServos(0);
                    currentState = AutoState.MOVE;
                    panelsTelemetry.addData("Status", "MOVE");
                }
                break;
            case MOVE:
                if (!pathStarted) {
                    follower.followPath(paths.Path1Line);
                    pathStarted = true;
                    panelsTelemetry.addData("Status", "Following Path");
                }
                follower.update();
                panelsTelemetry.addData("Position", "X: " + follower.getPose().getX() + ", Y: " + follower.getPose().getY());
//                if (!follower.isBusy()) {
//                    currentState = AutoState.DONE;
//                    pathStarted = false;
//                }
                break;
            case DONE:
                panelsTelemetry.addData("Status", "Done");
                break;
        }
    }

    private enum AutoState {
        REV_FLYWHEEL,
        BREAK1,
        RUN1,
        BREAK2,
        RUN2,
        RUN3,
        MOVE,
        DONE
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path1Line;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(23.968, 126.853),
                                    new Pose(69.759, 117.694),
                                    new Pose(60.016, 82.815)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(325))
                    .build();

            Path1Line = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(23.968, 126.853),
                                    new Pose(60.016, 82.815)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(140))
                    .setTranslationalConstraint(1.5)
                    .setHeadingConstraint(Math.toRadians(5))
                    .setVelocityConstraint(2)
                    .setTValueConstraint(0.98)
                    .setTimeoutConstraint(300)
                    .build();
        }
    }

}
