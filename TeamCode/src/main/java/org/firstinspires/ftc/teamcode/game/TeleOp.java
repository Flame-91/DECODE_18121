package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")
public class TeleOp extends OpMode {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private String team = "";

    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private FlywheelSubsystem flywheelSubsystem;

    private GamepadEx driver;
    private final TelemetryPacket telemetryPacket = new TelemetryPacket();


    @Override
    public void init() {
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry, telemetryPacket);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, telemetryPacket, dashboard);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap, telemetry, telemetryPacket);
        driver = new GamepadEx(gamepad1);

        mecanumDriveSubsystem.setDefaultCommand(
                new DriveCommand(driver, mecanumDriveSubsystem)
        );

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new LLAlignCommand(driver, mecanumDriveSubsystem, limelightSubsystem)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                () -> CommandScheduler.getInstance().schedule(
                        new FlywheelCommand(driver, flywheelSubsystem)
                )
        );
    }

    @Override
    public void init_loop() {
        if (gamepad1.right_bumper) team = "red";
        if (gamepad1.left_bumper) team = "blue";
        telemetry.addData("team", team);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();
    }
}
