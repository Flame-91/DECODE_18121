package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")

public class TeleOp extends OpMode {

    private MecanumDriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    public void init() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        driveSubsystem = new MecanumDriveSubsystem(hardwareMap, imu);
        driveCommand = new DriveCommand(gamepad1, driveSubsystem);
        CommandScheduler.getInstance().schedule(driveCommand);
    }
    public void init_loop() {}
    public void start() {}
    public void loop() {
        CommandScheduler.getInstance().run();
    }
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}