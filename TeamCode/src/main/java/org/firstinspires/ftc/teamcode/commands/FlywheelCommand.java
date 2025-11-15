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
    public static double flywheelMotorPower = 0.5575; // public static for panels
    public static double flywheelServoRuntime = 0.2;
    public static double flywheelServoBreaktime1 = 1;
    public static double flywheelServoBreaktime2 = 1.3;
    private final GamepadEx gamepad;
    private final ElapsedTime elapsedTime;
    private final ElapsedTime elapsedTime2;
    private boolean firstPress = true;
    private boolean hasReset = false;
    private boolean firstArtifact = false;

    public FlywheelCommand(GamepadEx gamepad, FlyWheelSubsystem flyWheelSubsystem) {
        this.gamepad = gamepad;
        this.flyWheelSubsystem = flyWheelSubsystem;
        elapsedTime = new ElapsedTime();
        elapsedTime2 = new ElapsedTime();
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .5) {
            flyWheelSubsystem.runFlywheel(flywheelMotorPower);
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .5) {
            flyWheelSubsystem.runFlywheel(-flywheelMotorPower);
        } else {
            flyWheelSubsystem.runFlywheel(0);
        }

        if (gamepad.getButton(GamepadKeys.Button.B)) {
            firstArtifact = false;
        }

        if (gamepad.getButton(GamepadKeys.Button.A)) {
            if (firstPress) {
                if (!hasReset) {
                    elapsedTime.reset();
                    hasReset = true;
                }
                flyWheelSubsystem.runFlywheelServos(1.0);
                if (elapsedTime.seconds() >= flywheelServoRuntime) {
                    firstPress = false;
                    hasReset = false;
                    flyWheelSubsystem.runFlywheelServos(0);
                    elapsedTime2.reset();
                }
            } else {
                if (elapsedTime2.seconds() >= flywheelServoBreaktime1) {
                    if (elapsedTime2.seconds() >= flywheelServoBreaktime1 && !firstArtifact) {
                        flyWheelSubsystem.runFlywheelServos(0);
                        firstPress = true;
                        hasReset = false;
                        firstArtifact = true;
                    } else if (elapsedTime2.seconds() >= flywheelServoBreaktime2 && firstArtifact) {
                        flyWheelSubsystem.runFlywheelServos(0);
                        firstPress = true;
                        hasReset = false;
                        firstArtifact = false;
                    }
                }
            }

            if (gamepad.getButton(GamepadKeys.Button.X)) {
                flyWheelSubsystem.runFlywheelServos(-1.0);
                firstPress = true;
                hasReset = false;
            } else {
                flyWheelSubsystem.runFlywheelServos(0);
                firstPress = true;
                hasReset = false;
            }
        }

//        @Override
//        public void end (boolean interrupted) {
//            flyWheelSubsystem.runFlywheel(0);
//            flyWheelSubsystem.runFlywheelServos(0);
//        }
    }
}