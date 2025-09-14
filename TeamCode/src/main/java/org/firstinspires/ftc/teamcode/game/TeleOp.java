package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.LLAlignCommand;
import org.firstinspires.ftc.teamcode.commands.LLGoToPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "game")

public class TeleOp extends OpMode {

    private MecanumDriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;
    private LLAlignCommand LLAlignCommand;
    private LLGoToPositionCommand LLGoToPositionCommand;

    public void init() {
        driveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        driveCommand = new DriveCommand(gamepad1, driveSubsystem);
        CommandScheduler.getInstance().schedule(driveCommand);
        if (gamepad1.a) {
            LLAlignCommand = new LLAlignCommand(driveSubsystem);
            CommandScheduler.getInstance().schedule(LLAlignCommand);
        }

//        if (gamepad1.x) {
//            LLGoToPositionCommand = new LLGoToPositionCommand(gamepad1, );
//        }
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