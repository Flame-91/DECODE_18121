package org.firstinspires.ftc.teamcode.game;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name = "CompAutoGoal")
@Configurable
public class CompAutoGoal extends OpMode {
    private static double breakTime = 3.25;
    private static double motorPower = 0.5575;
    private static double runTime = 0.2;
    private static double transferRunTime = 2;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    // Added a DONE state to signal the end of the autonomous sequence
    private enum ScoreState {
        BREAK0,
        RUN1,
        BREAK1,
        TRANSFER1,
        RUN2,
        BREAK2,
        RUN3,
        FORWARD,
        DONE// New state to signify completion
    }

    private ScoreState scoreState;
    private ElapsedTime elapsedTime;
    private FlyWheelSubsystem flywheelSubsystem;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void init() {
        elapsedTime = new ElapsedTime();
        scoreState = ScoreState.BREAK0;
        flywheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        // Only run the flywheel motor if we are not in the done state.
        if (scoreState != ScoreState.DONE) {
            flywheelSubsystem.runFlywheel(motorPower);
        }

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
                    scoreState = ScoreState.TRANSFER1;
                }
                break;

            case TRANSFER1:
                intakeSubsystem.setInnerIntakeServoPower(1);
                if (elapsedTime.seconds() >= transferRunTime) {
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
                    scoreState = ScoreState.FORWARD;
                }
                break;

            case FORWARD:
                frontLeft.setPower(1);
                backLeft.setPower(1);
                frontRight.setPower(1);
                backRight.setPower(1);
                if (elapsedTime.seconds() >= 3) {
                    scoreState = ScoreState.DONE;
                }
                break;

            case DONE:
                // Stop the flywheel motor and servos
                flywheelSubsystem.runFlywheel(0);
                flywheelSubsystem.runFlywheelServos(0);

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