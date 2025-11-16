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
    public static double flywheelMotorPower = 0.57067; // public static for panels
    public static double flywheelServoRuntime = 0.2;
    public static double flywheelServoBreaktime1 = 1;
    public static double flywheelServoBreaktime2 = 1.5;
    private final GamepadEx gamepad;
    private final ElapsedTime timer;
    private final ElapsedTime elapsedTime2;
    private boolean toggle = false;
    private boolean firstShotDone = false;

    public FlywheelCommand(GamepadEx gamepad, FlyWheelSubsystem flyWheelSubsystem) {
        this.gamepad = gamepad;
        this.flyWheelSubsystem = flyWheelSubsystem;
        timer = new ElapsedTime();
        elapsedTime2 = new ElapsedTime();
        addRequirements(flyWheelSubsystem);
    }

    @Override
    public void execute() {
        // ** Motor Logic **
        if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            toggle = !toggle;
        }
        if (toggle) {
            flyWheelSubsystem.runFlywheel(flywheelMotorPower);
        } else {
            flyWheelSubsystem.runFlywheel(0);
        }

        // ** Servo Logic **
        if (!gamepad.getButton(GamepadKeys.Button.A)) {
            firstShotDone = false;
            flyWheelSubsystem.runFlywheelServos(0);
            return;
        }
        if (!firstShotDone) {
            flyWheelSubsystem.runFlywheelServos(1);
            firstShotDone = true;
            timer.reset();
            return;
        }
        if (timer.seconds() > 1.5) {
            flyWheelSubsystem.runFlywheelServos(1);
            timer.reset();
        }
    }

//        @Override
//        public void end (boolean interrupted) {
//            flyWheelSubsystem.runFlywheel(0);
//            flyWheelSubsystem.runFlywheelServos(0);
//        }
    }