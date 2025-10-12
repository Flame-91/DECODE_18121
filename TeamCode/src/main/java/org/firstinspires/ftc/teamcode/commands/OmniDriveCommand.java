package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OmniDriveSubsystem;

public class OmniDriveCommand extends CommandBase {
    private final OmniDriveSubsystem omniDriveSubsystem;

    public OmniDriveCommand(OmniDriveSubsystem omniDriveSubsystem) {
        this.omniDriveSubsystem = omniDriveSubsystem;
        addRequirements(omniDriveSubsystem);
    }

    @Override
    public void execute() {
        omniDriveSubsystem.OmniDrive();
    }
}