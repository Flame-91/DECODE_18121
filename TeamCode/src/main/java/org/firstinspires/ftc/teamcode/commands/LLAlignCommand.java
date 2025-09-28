package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.util.PIDConstants.LLAlignKD;
import static org.firstinspires.ftc.teamcode.util.PIDConstants.LLAlignKI;
import static org.firstinspires.ftc.teamcode.util.PIDConstants.LLAlignKP;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class LLAlignCommand extends CommandBase {
    private MecanumDriveSubsystem drive;
//    private final double Kp = 0.001;       // proportional gain
//    private final double Ki = 0.01; // Integral gain
//    private final double Kd = 0.1; // Derivative gain
    private final double setpoint = 0;
    private final double maxYawSpeed = 0.7; // max rotation speed
//    double yaw;
    long lastTime = System.nanoTime();
    double output;
    private Gamepad gamepad;
    private Telemetry telemetry;
    PIDController PID = new PIDController(LLAlignKP, LLAlignKI, LLAlignKD, setpoint, maxYawSpeed); // Initialize pid controller
    private final LimelightSubsystem ll;
    double error = 0;

    public LLAlignCommand(MecanumDriveSubsystem drive, LimelightSubsystem ll, Gamepad gamepad, Telemetry telemetry) {
        this.drive = drive;
        this.ll = ll;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (ll.hasTarget()) {
            error = ll.getYawError(); // horizontal offset
            long currentTime = System.nanoTime();
            double deltaTime =  (currentTime - lastTime) / 1_000_000_000.0;
            lastTime = currentTime;

            output = PID.calculate(error, deltaTime);

            drive.drive(0, 0, output);

            telemetry.addData("Yaw Error", error);
            telemetry.addData("Yaw Correction", output);
            telemetry.update();
        }

    }

    @Override
    public boolean isFinished() {
        double tolerance = 2.0; // degrees tolerance
        if (!gamepad.a) {
            return true;
        } else {
            return Math.abs(error) < tolerance;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0); // stop rotation
    }
}
