package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

@Configurable
public class FlywheelCommand extends CommandBase {
    private final FlyWheelSubsystem flyWheelSubsystem;
    public static double flywheelMotorPower1A = 0.47; // public static for panels
    public static double flywheelMotorPower2A = 0.7;
    public static double flywheelMotorPower3A = 0.8;
    public static double flywheelServoRuntimeA = 0.2;
    public static double flywheelServoBreaktime1A = 0.75;
    public static double flywheelServoBreaktime2A = 1;
    private final GamepadEx gamepad;
    private final ElapsedTime servoElapsedTime;
    private final ElapsedTime motorElapsedTime;
    private enum ServoState { IDLE, REVERSE, RUN1, BREAK1, RUN2, BREAK2, RUN3 }
    private ServoState currentState = ServoState.IDLE;

    public FlywheelCommand(GamepadEx gamepad, FlyWheelSubsystem flyWheelSubsystem) {
        this.gamepad = gamepad;
        this.flyWheelSubsystem = flyWheelSubsystem;
        servoElapsedTime = new ElapsedTime();
        motorElapsedTime = new ElapsedTime();
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .5) {
            flyWheelSubsystem.runFlywheel(flywheelMotorPower1A);
            if (motorElapsedTime.seconds() > 2.75) {
                gamepad.gamepad.rumble(1, 1, 100);
                motorElapsedTime.reset();
            }
        } else {
            flyWheelSubsystem.runFlywheel(0);
            motorElapsedTime.reset();
        }
//        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .5) {
//            flyWheelSubsystem.runFlywheel(flywheelMotorPower);
//            if (motorElapsedTime.seconds() > 2.75) {
//                gamepad.gamepad.rumble(1, 1, 100);
//                motorElapsedTime.reset();
//            }
//        } else {
//            flyWheelSubsystem.runFlywheel(0);
//            motorElapsedTime.reset();
//        }

        gamepad.readButtons();

        if (gamepad.getButton(GamepadKeys.Button.Y)) {
            flyWheelSubsystem.runFlywheelServos(1.0);
            return;
        }

        switch (currentState) {
            case IDLE:
                flyWheelSubsystem.runFlywheelServos(0);
                if (gamepad.getButton(GamepadKeys.Button.A)) {
                    servoElapsedTime.reset();
                    currentState = ServoState.RUN1;
                }
                if (gamepad.getButton(GamepadKeys.Button.X)) {
                    servoElapsedTime.reset();
                    currentState = ServoState.REVERSE;
                }
                break;
            case RUN1:
                flyWheelSubsystem.runFlywheelServos(1);
                if (servoElapsedTime.seconds() >= flywheelServoRuntimeA) {
                    servoElapsedTime.reset();
                    currentState = ServoState.BREAK1;
                }
                break;
            case BREAK1:
                flyWheelSubsystem.runFlywheelServos(0);
                flyWheelSubsystem.runFlywheel(flywheelMotorPower2A);
                if (servoElapsedTime.seconds() >= flywheelServoBreaktime1A) {
                    servoElapsedTime.reset();
                    currentState = ServoState.RUN2;
                }
                break;
            case RUN2:
                flyWheelSubsystem.runFlywheelServos(1);
                flyWheelSubsystem.runFlywheel(flywheelMotorPower2A);
                if (servoElapsedTime.seconds() >= flywheelServoRuntimeA) {
                    servoElapsedTime.reset();
                    currentState = ServoState.BREAK2;
                }
                break;
            case BREAK2:
                flyWheelSubsystem.runFlywheelServos(0);
                flyWheelSubsystem.runFlywheel(flywheelMotorPower3A);
                if (servoElapsedTime.seconds() >= flywheelServoBreaktime2A) {
                    servoElapsedTime.reset();
                    currentState = ServoState.RUN3;
                }
                break;
            case RUN3:
                flyWheelSubsystem.runFlywheelServos(1);
                flyWheelSubsystem.runFlywheel(flywheelMotorPower3A);
                if (servoElapsedTime.seconds() >= flywheelServoRuntimeA) {
                    servoElapsedTime.reset();
                    currentState = ServoState.IDLE;
                }
                break;
            case REVERSE:
                flyWheelSubsystem.runFlywheelServos(-1);
                flyWheelSubsystem.runFlywheel(-1);
                if (!gamepad.getButton(GamepadKeys.Button.X)) currentState = ServoState.IDLE;
                break;
        }
    }

    public double getTimes() {
        return servoElapsedTime.seconds();
    }

    @Override
    public void end (boolean interrupted) {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }
}