package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class PivotCommand extends CommandBase {
    private final PivotSubsystem pivotSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final GamepadEx gamepad;
    private final PIDController pivotPIDController;

    public PivotCommand(GamepadEx gamepad, PivotSubsystem pivotSubsystem, LimelightSubsystem limelightSubsystem) {
        this.gamepad = gamepad;
        this.pivotSubsystem = pivotSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        pivotPIDController =

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {

    }
}
