package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TankDriveSubsystem;

public class TankDriveCommand extends CommandBase {
    private final TankDriveSubsystem drive;
    private final Gamepad gamepad;

    public TankDriveCommand(Gamepad gamepad, TankDriveSubsystem drive) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double y = -gamepad.left_stick_y; // Forward/backward
        double rotation = -gamepad.right_stick_x; // TurTeleOpning
        drive.drive(0, y, rotation); // x is ignored in tank drive
    }
}
