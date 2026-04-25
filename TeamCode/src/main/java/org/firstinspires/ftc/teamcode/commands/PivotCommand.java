package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKD;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKF;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKI;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotKP;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.pivotShootingOffset;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class PivotCommand extends CommandBase {
    private final PivotSubsystem pivotSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final PIDController pivotPIDController;
    private final GamepadEx gamepad;
    long lastTime = System.nanoTime();

    public PivotCommand(GamepadEx gamepad, PivotSubsystem pivotSubsystem, LimelightSubsystem limelightSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        double maxPivotSpeed = 1;
        pivotPIDController = new PIDController(pivotKP, pivotKI, pivotKD, maxPivotSpeed);

        this.gamepad = gamepad;

        addRequirements(this.pivotSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            pivotSubsystem.movePivotWithoutEncoder(-0.3);
        } if (gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            pivotSubsystem.resetPivotEncoder();
        } else if (limelightSubsystem.hasTarget()) {
            double pivotPositionAngle = pivotSubsystem.getCurrentPivotPositionAngle();
            double targetPosition = limelightSubsystem.getPitchErrorDegrees(pivotShootingOffset);
            double pitchError = targetPosition - pivotPositionAngle;

            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;
            lastTime = currentTime;
            double output = pivotPIDController.calculate(pitchError, deltaTime);
            double feedForward = pivotKF * Math.cos(Math.toRadians(targetPosition));

            double totalOutput = output +  feedForward;
            pivotSubsystem.setPivotPower(totalOutput);
        }
    }
}
