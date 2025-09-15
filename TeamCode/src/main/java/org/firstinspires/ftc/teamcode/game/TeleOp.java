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
    public final double[] redBaseCenterCoordinates = new double[]{0.9914, -0.8386}; // coords x,y in meters
    public final double[] blueBaseCenterCoordinates = new double[]{0.9914, 0.8386};
    public final double goToPositionPositionWithLLTolerance = 0.02;
    private String team;

    public void init() {
        driveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        driveCommand = new DriveCommand(gamepad1, driveSubsystem);
        CommandScheduler.getInstance().schedule(driveCommand);

        if (team.isEmpty()) {
            if (gamepad1.right_bumper) team = "red";
            if (gamepad1.left_bumper) team = "blue";
        }
    }
    public void init_loop() {}
    public void start() {}
    public void loop() {
        if (gamepad1.a) {
            LLAlignCommand = new LLAlignCommand(driveSubsystem);
            CommandScheduler.getInstance().schedule(LLAlignCommand);
        }

        if (gamepad1.x) {
            if (team == "blue") LLGoToPositionCommand = new LLGoToPositionCommand(driveSubsystem, blueBaseCenterCoordinates[0], blueBaseCenterCoordinates[1], goToPositionPositionWithLLTolerance);
            if (team == "red") LLGoToPositionCommand = new LLGoToPositionCommand(driveSubsystem, redBaseCenterCoordinates[0], redBaseCenterCoordinates[1], goToPositionPositionWithLLTolerance);
            CommandScheduler.getInstance().schedule(LLGoToPositionCommand);
        }
        CommandScheduler.getInstance().run();
    }
    public void stop() {
        CommandScheduler.getInstance().cancelAll();
    }
}