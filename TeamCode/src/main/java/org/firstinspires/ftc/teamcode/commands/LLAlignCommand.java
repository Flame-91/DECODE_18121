package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
//import org.firstinspires.ftc.teamcode.util.LinearController;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class LLAlignCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final double tolerance = 2.0;      // degrees tolerance
    private final double Kp = 0.001;       // proportional gain
    private final double Ki = 0.01; // Integral gain
    private final double Kd = 0.1; // Derivative gain
    private final double setpoint = 0;
    private final double maxYawSpeed = 0.2; // max rotation speed
//    double yaw;
    long lastTime = System.nanoTime();
    double output;
    PIDController PID = new PIDController(Kp, Ki, Kd, setpoint);
    HardwareMap hwMap = hardwareMap;
    LimelightSubsystem ll = new LimelightSubsystem(hwMap);
    double processVariable = 0;

    public LLAlignCommand(MecanumDriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
//        if (ll != null) { it won't be null bc we assign it a value
        if (ll.hasTarget()) {
            processVariable = ll.getYaw(); // horizontal offset
            long currentTime = System.nanoTime();
            double deltaTime =  (currentTime - lastTime) / 1_000_000_000.0;
            lastTime = currentTime;
            //        LC.setSetpoint(0);

            // PID control
            output = PID.calculate(processVariable, deltaTime);

            // Clamp to max rotation speed
            //        if (yawCorrection > maxYawSpeed) yawCorrection = maxYawSpeed; <-- already did in LinearControl method
            //        if (yawCorrection < -maxYawSpeed) yawCorrection = -maxYawSpeed;

            // Apply rotation
            drive.drive(0, 0, output); // NOT negative to correct direction bc it already makes it negative in LC method

            telemetry.addData("Yaw", processVariable);
            telemetry.addData("Yaw Correction", output);
            telemetry.update();
        }

    }

    @Override
    public boolean isFinished() {
        // DO NOT Keep command alive if no target
//        if (yaw == -361.0) return false;

        // Finish when aligned
        return Math.abs(processVariable) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0); // stop rotation
    }
}