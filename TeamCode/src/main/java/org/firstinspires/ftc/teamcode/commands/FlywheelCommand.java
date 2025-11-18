package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

@Config
@Configurable
public class FlywheelCommand extends CommandBase {
    private final FlywheelSubsystem flywheelSubsystem;
    private final GamepadEx gamepad;
    private enum FlywheelState {IDLE, RUN1, BREAK1, RUN2, BREAK2, RUN3, REVERSE}
    private FlywheelState currentState;
    private final ElapsedTime flywheelServoTimer;
    private final ElapsedTime flywheelMotorTimer;

    public final static double flywheelMotorPower = 1;
    public final static double flywheelMotorRuntime = 2.75;
    public final static double flywheelServoRuntime = 0.2;
    public final static double flywheelServoBreaktime1 = 0.75;
    public final static double flywheelServoBreaktime2 = 1.25;

    public FlywheelCommand(GamepadEx gamepad, FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.gamepad = gamepad;
        currentState = FlywheelState.IDLE;
        flywheelServoTimer = new ElapsedTime();
        flywheelMotorTimer = new ElapsedTime();

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            flywheelSubsystem.setFlywheelMotorPower(flywheelMotorPower);
            if (flywheelMotorTimer.seconds() >= flywheelMotorRuntime) {
                gamepad.gamepad.rumble(1, 1, 300);
                flywheelMotorTimer.reset();
            }
        } else {
            flywheelMotorTimer.reset();
        }

        switch (currentState) {
            case IDLE:
                flywheelSubsystem.setFlywheelServoPower(0);
                if (gamepad.getButton(GamepadKeys.Button.A)) {
                    currentState = FlywheelState.RUN1;
                    flywheelServoTimer.reset();
                }
                if (gamepad.getButton(GamepadKeys.Button.X)) {
                    currentState = FlywheelState.REVERSE;
                }
                break;

            case RUN1:
                flywheelSubsystem.setFlywheelServoPower(1);
                if (flywheelServoTimer.seconds() >= flywheelServoRuntime) {
                    currentState = FlywheelState.BREAK1;
                    flywheelServoTimer.reset();
                }
                break;

            case BREAK1:
                flywheelSubsystem.setFlywheelServoPower(0);
                if (flywheelServoTimer.seconds() >= flywheelServoBreaktime1) {
                    currentState = FlywheelState.RUN2;
                    flywheelServoTimer.reset();
                }
                break;

            case RUN2:
                flywheelSubsystem.setFlywheelServoPower(1);
                if (flywheelServoTimer.seconds() >= flywheelServoRuntime) {
                    currentState = FlywheelState.BREAK2;
                    flywheelServoTimer.reset();
                }
                break;

            case BREAK2:
                flywheelSubsystem.setFlywheelServoPower(0);
                if (flywheelServoTimer.seconds() >= flywheelServoBreaktime2) {
                    currentState = FlywheelState.RUN3;
                    flywheelServoTimer.reset();
                }
                break;

            case RUN3:
                flywheelSubsystem.setFlywheelServoPower(1);
                if (flywheelServoTimer.seconds() >= flywheelServoRuntime) {
                    currentState = FlywheelState.IDLE;
                    flywheelServoTimer.reset();
                }
                break;

            case REVERSE:
                flywheelSubsystem.setFlywheelServoPower(-1);
                flywheelSubsystem.setFlywheelMotorPower(-flywheelMotorPower);
                if (!gamepad.getButton(GamepadKeys.Button.X)) currentState = FlywheelState.IDLE;
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.setFlywheelMotorPower(0);
        flywheelSubsystem.setFlywheelServoPower(0);
    }
}
