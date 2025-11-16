package org.firstinspires.ftc.teamcode.game;

import static org.firstinspires.ftc.teamcode.commands.FlywheelCommand.flywheelMotorPower;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

@Autonomous(name = "CompAutoGoal")
@Configurable
public class CompAutoGoal extends OpMode {
    private static double breakTime = 3.25;
    private static double motorPower = 0.5575;
    private static double runTime = 0.2;
    // Added a DONE state to signal the end of the autonomous sequence
    private enum ScoreState {
        BREAK0,
        RUN1,
        BREAK1,
        RUN2,
        BREAK2,
        RUN3,
        DONE // New state to signify completion
    }

    private ScoreState scoreState;
    private ElapsedTime elapsedTime;
    private FlyWheelSubsystem flywheelSubsystem;

    @Override
    public void init() {
        // Initialize members in init()
        elapsedTime = new ElapsedTime();
        scoreState = ScoreState.BREAK0;
        flywheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        // Only run the flywheel motor if we are not in the DONE state.
        if (scoreState != ScoreState.DONE) {
            flywheelSubsystem.runFlywheel(motorPower);
        }

        switch (scoreState) {
            case BREAK0:
                // FIX: You MUST command the servos to their initial state.
                // This ensures they are at position 0 before the first "RUN" state.
                flywheelSubsystem.runFlywheelServos(0);

                if (elapsedTime.seconds() >= breakTime) { // Wait for flywheel spin-up
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
                    // Transition to the DONE state after the final action
                    scoreState = ScoreState.DONE;
                }
                break;

            case DONE:
                // Stop the flywheel motor and servos
                flywheelSubsystem.runFlywheel(0);
                flywheelSubsystem.runFlywheelServos(0);

                // End the OpMode gracefully after completion
                requestOpModeStop();
                break;
        }

        // Optional: Provide feedback on the current state
        telemetry.addData("Score State", scoreState);
        telemetry.addData("Time in State", elapsedTime.seconds());
        telemetry.update();
    }

    // Good practice: Stop motors when the OpMode ends (like when requestOpModeStop() is called)
    @Override
    public void stop() {
        if (flywheelSubsystem != null) {
            flywheelSubsystem.runFlywheel(0);
            flywheelSubsystem.runFlywheelServos(0);
        }
    }
}