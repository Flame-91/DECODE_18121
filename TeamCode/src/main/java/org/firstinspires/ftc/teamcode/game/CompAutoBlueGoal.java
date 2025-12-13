package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.FlywheelCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous(name = "CompAutoBlueGoal")
public class CompAutoBlueGoal extends OpMode {
    double motorPower = FlywheelCommand.flywheelMotorPower1A;
    double servoRun = FlywheelCommand.flywheelServoRuntimeA;
    double break1 = FlywheelCommand.flywheelServoBreaktime1A;
    double break2 = FlywheelCommand.flywheelServoBreaktime2A;
    double runTime = FlywheelCommand.flywheelServoRuntimeA;
    double revTime = 3.5;
    double delay = 1.0;
    double moveTime = 2.5;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private FlyWheelSubsystem flyWheelSubsystem;
    private ElapsedTime elapsedTime;
    private enum ScoreState {
        REV,
        RUN1,
        BREAK1,
        RUN2,
        BREAK2,
        RUN3,
        DELAY,
        MOVE,
        DONE
    }
    ScoreState scoreState;
    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
        scoreState = ScoreState.REV;
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        if (scoreState != ScoreState.MOVE && scoreState != ScoreState.DONE) {
            flyWheelSubsystem.runFlywheel(motorPower);
        }

        switch (scoreState) {
            case REV:
                flyWheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= revTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.RUN1;
                }
                break;

            case RUN1:
                flyWheelSubsystem.runFlywheelServos(1.0);
                if (elapsedTime.seconds() >= runTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.BREAK1;
                }
                break;

            case BREAK1:
                flyWheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= break1) {
                    elapsedTime.reset();
                    scoreState = ScoreState.RUN2;
                }
                break;

            case RUN2:
                flyWheelSubsystem.runFlywheelServos(1.0);
                if (elapsedTime.seconds() >= runTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.BREAK2;
                }
                break;

            case BREAK2:
                flyWheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= break2) {
                    elapsedTime.reset();
                    scoreState = ScoreState.RUN3;
                }
                break;

            case RUN3:
                flyWheelSubsystem.runFlywheelServos(1.0);
                if (elapsedTime.seconds() >= runTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.DELAY;
                }
                break;

            case DELAY:
                flyWheelSubsystem.runFlywheelServos(1.0);
                if (elapsedTime.seconds() >= delay) {
                    elapsedTime.reset();
                    scoreState = ScoreState.MOVE;
                }
                break;

            case MOVE:
                flyWheelSubsystem.runFlywheelServos(0);
                mecanumDriveSubsystem.drive(-0.25, -0.4, -0.25, -0.4);
                if (elapsedTime.seconds() >= moveTime) {
                    elapsedTime.reset();
                    scoreState = ScoreState.DONE;
                }
                break;

            case DONE:
                flyWheelSubsystem.runFlywheelServos(0);
                flyWheelSubsystem.runFlywheel(0);
                mecanumDriveSubsystem.drive(0, 0, 0, 0);
                requestOpModeStop();
                break;
        }

        telemetry.addData("Score State", scoreState);
        telemetry.addData("Time in State", elapsedTime.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (flyWheelSubsystem != null) {
            flyWheelSubsystem.runFlywheel(0);
            flyWheelSubsystem.runFlywheelServos(0);
        }
        if (mecanumDriveSubsystem != null) {
            mecanumDriveSubsystem.drive(0, 0, 0, 0);
        }
    }
}