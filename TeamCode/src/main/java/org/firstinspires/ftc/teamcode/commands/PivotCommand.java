package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.ConfigConstants.pivotKD;
import static org.firstinspires.ftc.teamcode.util.ConfigConstants.pivotKI;
import static org.firstinspires.ftc.teamcode.util.ConfigConstants.pivotKP;

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
            double pivotPositionAngle = pivotSubsystem.convertPivotTicksToAngle(pivotSubsystem.getCurrentPivotPosition());
            double pitchError = limelightSubsystem.getPitchError(0.42545) - pivotPositionAngle; // 0.42545 is how far up from the center of the april tag we need to shoot

            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;
            lastTime = currentTime;
            double output = pivotPIDController.calculate(pitchError, deltaTime);

            pivotSubsystem.setPivotTargetPosition(pivotSubsystem.convertPivotAngleToTicks(pitchError));
            pivotSubsystem.setPivotPower(-output);
        }
    }
}
