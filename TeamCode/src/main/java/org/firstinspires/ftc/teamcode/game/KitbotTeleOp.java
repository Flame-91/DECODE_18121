package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
//import com.qualcomm.robotcore.util.ElapsedTime;

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

    @Override
    public void init() {
        omniDriveSubsystem = new OmniDriveSubsystem(hardwareMap, gamepad1);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, gamepad1);
        omniDriveCommand = new OmniDriveCommand(omniDriveSubsystem);
        flyWheelCommand = new FlyWheelCommand(flyWheelSubsystem);
        CommandScheduler.getInstance().schedule(omniDriveCommand, flyWheelCommand);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}