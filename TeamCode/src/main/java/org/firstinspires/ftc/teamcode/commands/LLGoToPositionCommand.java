package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robocol.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class LLGoToPositionCommand extends CommandBase {
    LimelightSubsystem ll;
    final private double xTarget;
    final private double yTarget;
    private final MecanumDriveSubsystem drive;
    private final Gamepad gamepad;
    //    final private double
    public LLGoToPositionCommand(Gamepad gamepad, MecanumDriveSubsystem drive, double xTarget, double yTarget) {
        this.xTarget = xTarget;
        this.yTarget = yTarget;
        this.drive = drive;
        this.gamepad = gamepad;
    }


}
