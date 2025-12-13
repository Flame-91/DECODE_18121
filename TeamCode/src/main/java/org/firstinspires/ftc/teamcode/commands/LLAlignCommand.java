package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.GlobalConstants.LLAlignKD;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.LLAlignKI;
import static org.firstinspires.ftc.teamcode.util.GlobalConstants.LLAlignKP;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class LLAlignCommand extends CommandBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    public double[] periodicMT2Pos;
    private final GamepadEx gamepad;

    PIDController PID; // Initialize pid controller
    long lastTime = System.nanoTime();
    double output;
    double error = 0;

    public LLAlignCommand(GamepadEx gamepad, MecanumDriveSubsystem mecanumDriveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.gamepad = gamepad;
        double maxYawSpeed = 1;
        PID = new PIDController(LLAlignKP, LLAlignKI, LLAlignKD, maxYawSpeed);

        addRequirements(this.mecanumDriveSubsystem);
    }

    @Override
    public void execute() {
        if (limelightSubsystem.hasTarget()) {
            error = limelightSubsystem.getYawErrorDegrees() + 6; // horizontal offset, negative since error is defined as target - current which is 0 - yawError = -yawError
            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

            lastTime = currentTime;

            output = PID.calculate(error, deltaTime);

            mecanumDriveSubsystem.drive(0, 0, output);
        }
    }

    @Override
    public boolean isFinished() {
        double tolerance = 10.0; // degrees tolerance
        if (!gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            return true;
        } else {
            return Math.abs(error) < tolerance;
        }
    }

    @Override
    public void end(boolean interrupted) {mecanumDriveSubsystem.drive(0, 0, 0);}
}