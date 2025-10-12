package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.ConfigConstants.LLAlignKD;
import static org.firstinspires.ftc.teamcode.util.ConfigConstants.LLAlignKI;
import static org.firstinspires.ftc.teamcode.util.ConfigConstants.LLAlignKP;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class LLAlignCommand extends CommandBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final LimelightSubsystem limelightSubsystem;

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

        addRequirements((Subsystem) this.mecanumDriveSubsystem);
    }

    @Override
    public void execute() {
        if (limelightSubsystem.hasTarget()) {
            error = limelightSubsystem.getYawError(); // horizontal offset
            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1_000_000_000.0;

            lastTime = currentTime;

            output = PID.calculate(error, deltaTime);

            mecanumDriveSubsystem.drive(0, 0, -output);
        }
    }

    @Override
    public boolean isFinished() {
        double tolerance = 10.0; // degrees tolerance
        if (!gamepad.getButton(GamepadKeys.Button.A)) {
            return true;
        } else {
            return Math.abs(error) < tolerance;
        }
    }

    @Override
    public void end(boolean interrupted) {mecanumDriveSubsystem.drive(0, 0, 0);}
}