package org.firstinspires.ftc.teamcode.game;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlywheelCommandAutomatic;
import org.firstinspires.ftc.teamcode.commands.FlywheelCommandManual;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
@TeleOp (name = "Kit Bot TeleOp")
public class KitBotTeleOp extends OpMode {
    FlyWheelSubsystem flyWheelSubsystem;
    MecanumDriveSubsystem mecanumDriveSubsystem;
    DriveCommand driveCommand;
    FlywheelCommandManual flywheelCommandManual;

    @Override
    public void init() {
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        flywheelCommandManual = new FlywheelCommandManual((new GamepadEx(gamepad1)), flyWheelSubsystem);
        driveCommand = new DriveCommand((new GamepadEx(gamepad1)), mecanumDriveSubsystem);
        mecanumDriveSubsystem.setDefaultCommand(driveCommand);
        flyWheelSubsystem.setDefaultCommand(flywheelCommandManual);
    }

    @Override
    public void loop() {
        telemetry.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
    }
}
