package org.firstinspires.ftc.teamcode.game;

//import com.pedropathing.ftc.drivetrains.Mecanum;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FlywheelCommandAutomatic;
import org.firstinspires.ftc.teamcode.commands.FlywheelCommandManual;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
@TeleOp (name = "Kit Bot TeleOp")
public class KitBotTeleOp extends OpMode {
    FlyWheelSubsystem flyWheelSubsystem;
//    OmniDriveSubsystemOG omniDriveSubsystem;
    MecanumDriveSubsystem mecanumDriveSubsystem;
    DriveCommand driveCommand;
//    FlywheelCommandAutomatic flywheelCommandAutomatic;
    FlywheelCommandManual flywheelCommandManual;

    @Override
    public void init() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(imuParams);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap, telemetry);
//        omniDriveSubsystem = new OmniDriveSubsystemOG(hardwareMap, gamepad1, telemetry);
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, imu, telemetry);
//        flywheelCommandAutomatic = new FlywheelCommandAutomatic((new GamepadEx(gamepad1)), flyWheelSubsystem);
        flywheelCommandManual = new FlywheelCommandManual((new GamepadEx(gamepad1)), flyWheelSubsystem);
        driveCommand = new DriveCommand((new GamepadEx(gamepad1)), mecanumDriveSubsystem);
        mecanumDriveSubsystem.setDefaultCommand(driveCommand);
//        flyWheelSubsystem.setDefaultCommand(flywheelCommandAutomatic);
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
