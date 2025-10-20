package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

public class FlywheelCommand extends CommandBase {
    private final FlyWheelSubsystem flyWheelSubsystem;
    private final GamepadEx gamepad;
    private final ElapsedTime elapsedTime;
    private boolean firstButtonPress;
    private boolean hasReset;
    private boolean activateServos;

    public FlywheelCommand(GamepadEx gamepad, FlyWheelSubsystem flyWheelSubsystem) {
        this.gamepad = gamepad;
        this.flyWheelSubsystem = flyWheelSubsystem;
        firstButtonPress = true;
        hasReset = false;
        activateServos = false;
        elapsedTime = new ElapsedTime();
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        boolean a = gamepad.getButton(GamepadKeys.Button.A);
        boolean x = gamepad.getButton(GamepadKeys.Button.X);

        if (a) {
            flyWheelSubsystem.runFlywheel(1.0);
            if (firstButtonPress) {
                elapsedTime.reset();
                hasReset = true;
            }
            if (hasReset && elapsedTime.seconds() > 1) activateServos = true;
            if (activateServos) flyWheelSubsystem.runFlywheelServos(1.0);
            firstButtonPress = false;
        } else if (x) {
            firstButtonPress = true;
            hasReset = false;
            activateServos = false;
            flyWheelSubsystem.runFlywheel(-0.25);
            flyWheelSubsystem.runFlywheelServos(-.6);
        } else {
            firstButtonPress = true;
            hasReset = false;
            activateServos = false;
            flyWheelSubsystem.runFlywheel(0);
            flyWheelSubsystem.runFlywheelServos(0);
        }
    }
}