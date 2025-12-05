package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.FlywheelCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Paths;

@Autonomous(name = "CompAutoGoal")
@Configurable
public class CompAutoGoal extends OpMode {
    private static double breakTime = 3;
    private static double motorPower = 0.4867;
    private static double runTime = FlywheelCommand.flywheelServoRuntime;
    Follower follower;
    Paths paths;
    MecanumDriveSubsystem mecanumDriveSubsystem;
    private boolean pathStarted = false;
    // Added a DONE state to signal the end of the autonomous sequence
    private enum ScoreState {
        BREAK0,
        RUN1,
        BREAK1,
        RUN2,
        BREAK2,
        RUN3,
        DELAY,
        DONE,
        MOVE1,
        MOVE2// New state to signify completion
    }

    private ScoreState scoreState;
    private ElapsedTime elapsedTime;
    private FlyWheelSubsystem flywheelSubsystem;

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
        scoreState = ScoreState.BREAK0;
        flywheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22, 124, Math.toRadians(320)));
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        paths = new Paths(follower);
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        // Only run the flywheel motor if we are not in the done state.
        if (scoreState != ScoreState.MOVE1 && scoreState != ScoreState.MOVE2) {
            flywheelSubsystem.runFlywheel(motorPower);
        }
        follower.update();

        switch (scoreState) {
            case BREAK0:
                flywheelSubsystem.runFlywheelServos(0);

                if (elapsedTime.seconds() >= breakTime) { // Wait for flywheel
                    elapsedTime.reset();
                    scoreState = ScoreState.RUN1;
                }
                break;

            case RUN1:
                flywheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= runTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.BREAK1;
                }
                break;

            case BREAK1:
                flywheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= breakTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.RUN2;
                }
                break;

            case RUN2:
                flywheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= runTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.BREAK2;
                }
                break;

            case BREAK2:
                flywheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= breakTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.RUN3;
                }
                break;

            case RUN3:
                flywheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= runTime) {
                    elapsedTime.reset();
                    pathStarted = false;
                    scoreState = ScoreState.DELAY;
                }
                break;

            case DELAY:
                if (elapsedTime.seconds() >= 1.5) {
                    pathStarted = false;
                    elapsedTime.reset();
                    scoreState = ScoreState.MOVE1;
                }
                break;

            case MOVE1:
                if (!pathStarted) {
                    follower.followPath(paths.goal_blue);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    elapsedTime.reset();
                    scoreState = ScoreState.DONE;
                }
                break;

            case MOVE2:
                if (!pathStarted) {
                    follower.followPath(paths.goal_blue2);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    elapsedTime.reset();
                    scoreState = ScoreState.DONE;
                }
                break;

            case DONE:
                // Stop the flywheel motor and servos
                flywheelSubsystem.runFlywheel(0);
                flywheelSubsystem.runFlywheelServos(0);
                mecanumDriveSubsystem.drive(0);

                requestOpModeStop();
                break;
        }


        telemetry.addData("Score State", scoreState);
        telemetry.addData("Time in State", elapsedTime.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (flywheelSubsystem != null) {
            flywheelSubsystem.runFlywheel(0);
            flywheelSubsystem.runFlywheelServos(0);
        }
    }
}