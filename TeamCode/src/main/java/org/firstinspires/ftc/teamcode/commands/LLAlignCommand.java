package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class LLAlignCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final double tolerance = 2.0;      // degrees tolerance
    private final double kP = 0.01;       // proportional gain
    private final double maxYawSpeed = 0.2; // max rotation speed
    double yaw;
    LimelightSubsystem LimelightSubsystem;

    public LLAlignCommand(MecanumDriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        yaw = LimelightSubsystem.getYaw(); // horizontal offset

        // Pause if no target
        if (yaw == -361.0) {
            drive.drive(0, 0, 0);
            return;
        }

        // Proportional control
        double yawCorrection = yaw * kP;

        // Clamp to max rotation speed
        if (yawCorrection > maxYawSpeed) yawCorrection = maxYawSpeed;
        if (yawCorrection < -maxYawSpeed) yawCorrection = -maxYawSpeed;

        // Apply rotation
        drive.drive(0, 0, -yawCorrection); // negative to correct direction

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Yaw Correction", yawCorrection);
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        // Keep command alive if no target
        if (yaw == -361.0) return false;

        // Finish when aligned
        return Math.abs(yaw) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0); // stop rotation
    }
}
