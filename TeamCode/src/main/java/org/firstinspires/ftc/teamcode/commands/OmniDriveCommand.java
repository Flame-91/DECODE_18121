package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OmniDriveSubsystem;

public class OmniDriveCommand extends CommandBase {
    private final OmniDriveSubsystem omniDriveSubsystem;
    private final Gamepad gamepad;

    public OmniDriveCommand(Gamepad gamepad, OmniDriveSubsystem omniDriveSubsystem) {
        this.omniDriveSubsystem = omniDriveSubsystem;
        this.gamepad = gamepad;
        addRequirements(omniDriveSubsystem);
    }

    @Override
    public void execute() {
        double forward = -gamepad.left_stick_y;
        double rotation = gamepad.right_stick_x;
        omniDriveSubsystem.OmniDrive(forward, rotation);
    }
}