package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.PIDConstants.pivotKD;
import static org.firstinspires.ftc.teamcode.util.PIDConstants.pivotKI;
import static org.firstinspires.ftc.teamcode.util.PIDConstants.pivotKP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class PivotCommand extends CommandBase {
    private final PivotSubsystem pivotSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final PIDController pivotPIDController;

    public PivotCommand(PivotSubsystem pivotSubsystem, LimelightSubsystem limelightSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        double maxPivotSpeed = 1;
        pivotPIDController = new PIDController(pivotKP, pivotKI, pivotKD, maxPivotSpeed);

        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void execute() {
        if (limelightSubsystem.hasTarget()) {

            pivotPIDController.
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
