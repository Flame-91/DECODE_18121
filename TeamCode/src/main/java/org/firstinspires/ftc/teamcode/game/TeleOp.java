package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")
public class TeleOp extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private GamepadEx driver;
    private final TelemetryPacket telemetryPacket = new TelemetryPacket();


    @Override
    public void init() {
        //Gamepad
        driver = new GamepadEx(gamepad1);

        //Subsystems
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry, telemetryPacket);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, telemetryPacket, dashboard);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, telemetryPacket);

        //Commands
        mecanumDriveSubsystem.setDefaultCommand(
                new DriveCommand(driver, mecanumDriveSubsystem)
        );

        intakeSubsystem.setDefaultCommand(
                new IntakeCommand(intakeSubsystem)
        );

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new LLAlignCommand(driver, mecanumDriveSubsystem, limelightSubsystem))
        );
    }

    @Override
    public void init_loop() {}

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetryPacket.put("Status", "Running TeleOp");
        dashboard.sendTelemetryPacket(telemetryPacket);
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
