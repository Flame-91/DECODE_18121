package org.firstinspires.ftc.teamcode.game;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.FlyWheelCommand;
import org.firstinspires.ftc.teamcode.commands.OmniDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ServoCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OmniDriveSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Kit Bot TeleOp")
public class TeleOp extends OpMode {
    private final JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

    OmniDriveSubsystem omniDriveSubsystem;
    FlyWheelSubsystem flyWheelSubsystem;
    OmniDriveCommand omniDriveCommand;
    FlyWheelCommand flyWheelCommand;
    ServoCommand servoCommand;
    boolean pressedA = false;
    boolean pressedX = false;

    @Override
    public void init() {
        omniDriveSubsystem = new OmniDriveSubsystem(hardwareMap, gamepad1);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);

        omniDriveCommand = new OmniDriveCommand(gamepad1, omniDriveSubsystem);
        flyWheelCommand = new FlyWheelCommand(gamepad1, flyWheelSubsystem);
        servoCommand = new ServoCommand(gamepad1, flyWheelSubsystem);

        omniDriveSubsystem.setDefaultCommand(omniDriveCommand);
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
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
        joinedTelemetry.addData("Status", "Stopped");
    }
}