package org.firstinspires.ftc.teamcode.game;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.FlyWheelCommand;
import org.firstinspires.ftc.teamcode.commands.ServoCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Kit Bot TeleOp")
public class TeleOp extends OpMode {
    private final JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    FlyWheelSubsystem flyWheelSubsystem;
    FlyWheelCommand flyWheelCommand;
    ServoCommand servoCommand;
    boolean pressedA = false;
    boolean pressedX = false;

    private GamepadEx driver;

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);

        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, joinedTelemetry);

        flyWheelCommand = new FlyWheelCommand(gamepad1, flyWheelSubsystem);
        servoCommand = new ServoCommand(gamepad1, flyWheelSubsystem);

        joinedTelemetry.addData("Status", "Running");
        joinedTelemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.x && !pressedX) {
            CommandScheduler.getInstance().schedule(servoCommand);
            pressedX = true;
        } else {
            pressedX = false;
        }

        if (gamepad1.a && !pressedA) {
            CommandScheduler.getInstance().schedule(flyWheelCommand);
            pressedA = true;
        } else {
            pressedA = false;
        }

        CommandScheduler.getInstance().run();
        joinedTelemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
        joinedTelemetry.addData("Status", "Stopped");
    }
}