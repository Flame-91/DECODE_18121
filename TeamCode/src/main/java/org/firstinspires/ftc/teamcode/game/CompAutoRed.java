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

// The BlocksOpModeCompanion imports for hardwareMap and telemetry are removed
// because OpMode already provides access to them.

@Autonomous(name="CompAutoRed")
public class CompAutoRed extends OpMode {
    // --- State Variables ---
    private AutoState currentState = AutoState.GO_TO_PRELOAD_SCORE;
    private ScoreState currentScoreState = ScoreState.RUN1;

    // --- Subsystems and Utilities ---
    FlyWheelSubsystem flyWheelSubsystem;
    Paths paths;
    ElapsedTime servoElapsedTime;
    Timer reloadTime;

    // --- Control Flags ---
    private int shotCount = 0; // Not currently used, but good for tracking
    private boolean pathStarted = false; // Needed to make the paths run once

    // --- Enums for State Machines ---
    private enum AutoState {
        GO_TO_PRELOAD_SCORE,
        DRIVE_TO_RELOAD,
        DRIVE_TO_SCORE,
        SCORE_1,
        IDLE
    }

    private enum ScoreState {
        RUN1,
        BREAK1,
        RUN2,
        BREAK2,
        RUN3
    }

    /**
     * Initialization method run once when INIT is pressed.
     */
    public void init() {
        servoElapsedTime = new ElapsedTime();
        reloadTime = new Timer();

        // Use the inherited hardwareMap and telemetry from OpMode
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);

        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
        paths = new Paths(follower);

        telemetry.addData("Status", "Initialized");
    }

    /**
     * Main loop run continuously after START is pressed.
     */
    public void loop() {
        // Keep the flywheel motor running constantly
        flyWheelSubsystem.runFlywheel(0.57067);

        // Update the pathing follower (mandatory)
        follower.update();

        // Run the state machine logic
        autoStateUpdate();

        // Telemetry updates for debugging
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("current AutoState: ", currentState);
        telemetry.addData("current ScoreState: ", currentScoreState);
        telemetry.update();
    }

    /**
     * Primary Autonomous State Machine (Drive/Sequence Logic)
     */
    private void autoStateUpdate() {
        switch (currentState) {
            // -- GO TO PRELOAD SCORE POSITION --
            case GO_TO_PRELOAD_SCORE:
                if (!pathStarted) {
                    // Start the path once
                    follower.followPath(paths.preload_score_red);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    // Path is complete
                    pathStarted = false;
                    servoElapsedTime.reset();
                    currentState = AutoState.SCORE_1;
                }
                break;

            //  -- SCORE RINGS (Initial 3 shots) --
            case SCORE_1:
                // scoreAuto() manages the servo state machine and returns true when 3 shots are done
                if (scoreAuto()) {
                    // All 3 shots are complete, move to reload
                    currentState = AutoState.DRIVE_TO_RELOAD;
                }
                break;

            // -- DRIVE TO RELOAD ZONE --
            case DRIVE_TO_RELOAD:
                if (!pathStarted) {
                    follower.followPath(paths.to_reload_red);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    // Start the timer while the robot waits/intakes
                    reloadTime.resetTimer();
                    currentState = AutoState.DRIVE_TO_SCORE;
                }
                break;

            // -- WAIT FOR RELOAD AND DRIVE BACK TO SCORE POSITION --
            case DRIVE_TO_SCORE:
                // Start the path only after the 2-second reload time AND if the path hasn't started
                if (!pathStarted && reloadTime.getElapsedTimeSeconds() > 2) {
                    follower.followPath(paths.to_score_red);
                    pathStarted = true;
                }

                // If the path has started and is now finished, transition to shooting again
                if (pathStarted && !follower.isBusy()) {
                    pathStarted = false;
                    // Note: You may want to score a second set of rings here,
                    // but since SCORE_2 and SCORE_3 were removed, it goes back to SCORE_1
                    currentState = AutoState.SCORE_1;
                }
                break;

            // -- IDLE STATE (Autonomous finished) --
            case IDLE:
                flyWheelSubsystem.runFlywheel(0); // Stop the flywheel
                break;
        }
    }

    /**
     * Servo State Machine for cycling the shooting mechanism 3 times.
     * @return true when the full 3-shot sequence is complete, false otherwise.
     */
    private boolean scoreAuto() {
        switch (currentScoreState) {
            case RUN1:
                flyWheelSubsystem.runFlywheelServos(1);
                if (servoElapsedTime.seconds() >= flywheelServoRuntime) {
                    servoElapsedTime.reset();
                    currentScoreState = ScoreState.BREAK1;
                }
                return false; // Not finished, keep running loop
            case BREAK1:
                flyWheelSubsystem.runFlywheelServos(0);
                if (servoElapsedTime.seconds() >= flywheelServoBreaktime1) {
                    servoElapsedTime.reset();
                    currentScoreState = ScoreState.RUN2;
                }
                return false; // Not finished
            case RUN2:
                flyWheelSubsystem.runFlywheelServos(1);
                if (servoElapsedTime.seconds() >= flywheelServoRuntime) {
                    servoElapsedTime.reset();
                    currentScoreState = ScoreState.BREAK2;
                }
                return false; // Not finished
            case BREAK2:
                flyWheelSubsystem.runFlywheelServos(0);
                if (servoElapsedTime.seconds() >= flywheelServoBreaktime2) {
                    servoElapsedTime.reset();
                    currentScoreState = ScoreState.RUN3;
                }
                return false; // Not finished
            case RUN3:
                flyWheelSubsystem.runFlywheelServos(1);
                if (servoElapsedTime.seconds() >= flywheelServoRuntime) {
                    servoElapsedTime.reset();
                    // Reset to the beginning of the sequence for the next set of rings
                    currentScoreState = ScoreState.RUN1;
                    shotCount++; // Increment shot count (if needed later)
                    return true; // Sequence of 3 shots is COMPLETE
                }
                return false; // Not finished yet
        }
        return false; // Default return (should not be reached)
    }

    @Override
    public void stop() {
        // Ensure the motor and servos stop when the OpMode is stopped
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }
}