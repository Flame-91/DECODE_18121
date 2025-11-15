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
    public static double flywheelMotorPower = 0.575; // public static for panels
    public static double flywheelServoRuntime = 0.2;
    public static double flywheelServoBreaktime1 = 1;
    public static double flywheelServoBreaktime2 = 1.5;
    private final GamepadEx gamepad;
    private final ElapsedTime elapsedTime;
    private boolean firstPress = true;
    private boolean hasReset = false;
    private boolean firstArtifact = false;
    private enum ServoState {IDLE, RUNNING1, RUNNING2, RUNNING3, REVERSE, BREAK1, BREAK2};
    private ServoState currentState = ServoState.IDLE;
    public FlywheelCommand(GamepadEx gamepad, FlyWheelSubsystem flyWheelSubsystem) {
        this.gamepad = gamepad;
        this.flyWheelSubsystem = flyWheelSubsystem;
        elapsedTime = new ElapsedTime();
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        gamepad.readButtons();

        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            flyWheelSubsystem.runFlywheel(flywheelMotorPower);
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            flyWheelSubsystem.runFlywheel(-flywheelMotorPower);
        } else {
            flyWheelSubsystem.runFlywheel(0);
        }

        elapsedTime.reset();

        switch (currentState) {
            case IDLE:
                flyWheelSubsystem.runFlywheelServos(0);
                if (gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                    elapsedTime.reset();
                    currentState = ServoState.RUNNING1;
                }
                if (gamepad.getButton(GamepadKeys.Button.X)) {
                    currentState = ServoState.REVERSE;
                }
                break;
            case RUNNING1:
                flyWheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= flywheelServoRuntime) {
                    elapsedTime.reset();
                    currentState = ServoState.BREAK1;
                }
                break;
            case BREAK1:
                flyWheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= flywheelServoBreaktime1) {
                    elapsedTime.reset();
                    currentState = ServoState.RUNNING2;
                }
                break;
            case RUNNING2:
                flyWheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= flywheelServoRuntime) {
                    elapsedTime.reset();
                    currentState = ServoState.BREAK2;
                }
            case BREAK2:
                flyWheelSubsystem.runFlywheelServos(0);
                if (elapsedTime.seconds() >= flywheelServoBreaktime1) {
                    elapsedTime.reset();
                    currentState = ServoState.RUNNING3;
                }
                break;
            case RUNNING3:
                flyWheelSubsystem.runFlywheelServos(1);
                if (elapsedTime.seconds() >= flywheelServoRuntime) {
                    elapsedTime.reset();
                    currentState = ServoState.IDLE;
                }
            case REVERSE:
                flyWheelSubsystem.runFlywheelServos(-1);
                if (!gamepad.getButton(GamepadKeys.Button.X)) {
                    currentState = ServoState.IDLE;
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        flyWheelSubsystem.runFlywheel(0);
        flyWheelSubsystem.runFlywheelServos(0);
    }
}