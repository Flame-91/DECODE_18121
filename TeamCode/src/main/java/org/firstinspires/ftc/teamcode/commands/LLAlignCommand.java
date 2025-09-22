package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class LLAlignCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final double Kp = 0.001;       // proportional gain
    private final double Ki = 0.01; // Integral gain
    private final double Kd = 0.1; // Derivative gain
    private final double setpoint = 0;
    private final double maxYawSpeed = 0.7; // max rotation speed
//    double yaw;
    long lastTime = System.nanoTime();
    double output;
    PIDController PID = new PIDController(Kp, Ki, Kd, setpoint, maxYawSpeed); // Initialize pid controller
    LimelightSubsystem ll = new LimelightSubsystem(hardwareMap);
    double error = 0;

    public LLAlignCommand(MecanumDriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (ll.hasTarget()) {
            error = ll.getYaw(); // horizontal offset
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
        return Math.abs(error) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0); // stop rotation
    }
}
