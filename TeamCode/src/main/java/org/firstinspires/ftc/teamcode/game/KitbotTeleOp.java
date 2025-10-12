package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.FlyWheelCommand;
import org.firstinspires.ftc.teamcode.commands.OmniDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OmniDriveSubsystem;

@TeleOp (name = "Kit Bot TeleOp")
public class KitbotTeleOp extends OpMode {
    OmniDriveSubsystem omniDriveSubsystem;
    FlyWheelSubsystem flyWheelSubsystem;
    OmniDriveCommand omniDriveCommand;
    FlyWheelCommand flyWheelCommand;
    boolean pressedA = false;

    @Override
    public void init() {
        omniDriveSubsystem = new OmniDriveSubsystem(hardwareMap, gamepad1);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);

        omniDriveCommand = new OmniDriveCommand(gamepad1, omniDriveSubsystem);
        flyWheelCommand = new FlyWheelCommand(gamepad1, flyWheelSubsystem);

        omniDriveSubsystem.setDefaultCommand(omniDriveCommand);
    }

    @Override
    public void loop() {
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
    }
}