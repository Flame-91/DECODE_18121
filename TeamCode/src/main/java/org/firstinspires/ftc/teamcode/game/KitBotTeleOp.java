package org.firstinspires.ftc.teamcode.game;

//import com.pedropathing.ftc.drivetrains.Mecanum;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlywheelCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.OmniDriveSubsystemOG;

@TeleOp (name = "Kit Bot TeleOp real")
public class KitBotTeleOp extends OpMode {
    FlyWheelSubsystem flyWheelSubsystem;
//    OmniDriveSubsystemOG omniDriveSubsystem;
    MecanumDriveSubsystem mecanumDriveSubsystem;
    DriveCommand driveCommand;
    FlywheelCommand flywheelCommand;

    @Override
    public void init() {
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
//        omniDriveSubsystem = new OmniDriveSubsystemOG(hardwareMap, gamepad1, telemetry);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        flywheelCommand = new FlywheelCommand((new GamepadEx(gamepad1)), flyWheelSubsystem);
        driveCommand = new DriveCommand((new GamepadEx(gamepad1)), mecanumDriveSubsystem);
        mecanumDriveSubsystem.setDefaultCommand(driveCommand);
        flyWheelSubsystem.setDefaultCommand(flywheelCommand);
    }

    @Override
    public void loop() {
        telemetry.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}
