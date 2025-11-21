package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlywheelCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
@TeleOp (name = "Kit Bot TeleOp")
public class KitBotTeleOp extends OpMode {
    FlyWheelSubsystem flyWheelSubsystem;
    MecanumDriveSubsystem mecanumDriveSubsystem;
    IntakeSubsystem intakeSubsystem;
    DriveCommand driveCommand;
    FlywheelCommand flywheelCommandManual;
    IntakeCommand intakeCommand;
    IMU imu;
    GamepadEx gamepadEx;
    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        gamepadEx = new GamepadEx(gamepad1);
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(imuParams);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        flywheelCommandManual = new FlywheelCommand(gamepadEx, flyWheelSubsystem);
        driveCommand = new DriveCommand(gamepadEx, mecanumDriveSubsystem);
        intakeCommand = new IntakeCommand(gamepadEx, intakeSubsystem);
        mecanumDriveSubsystem.setDefaultCommand(driveCommand);
        flyWheelSubsystem.setDefaultCommand(flywheelCommandManual);
        intakeSubsystem.setDefaultCommand(intakeCommand);
    }

    @Override
    public void loop() {
//        telemetry.addData("robotYaw: ", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("timers", flywheelCommandManual.getTimes());
        telemetry.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
    }
}
