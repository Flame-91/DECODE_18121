package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.ConfigConstants.pivotKD;
import static org.firstinspires.ftc.teamcode.util.ConfigConstants.pivotKI;
import static org.firstinspires.ftc.teamcode.util.ConfigConstants.pivotKP;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class PivotCommand extends CommandBase {
    private final PivotSubsystem pivotSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final PIDController pivotPIDController;
    long lastTime = System.nanoTime();

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
            double pitchError = limelightSubsystem.getPitchError(.2794);
            double pivotPositionAngle = pivotSubsystem.convertPivotTicksToAngle(pivotSubsystem.getCurrentPivotPosition());
            pitchError -= pivotPositionAngle;
            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;
            lastTime = currentTime;
            double output = pivotPIDController.calculate(pitchError, deltaTime);

            pivotSubsystem.setPivotPosition(pivotSubsystem.convertPivotAngleToTicks(pitchError));
            pivotSubsystem.setPivotPower(-output);
        }
    }

    @Override
    public boolean isFinished() { return false; }
}
