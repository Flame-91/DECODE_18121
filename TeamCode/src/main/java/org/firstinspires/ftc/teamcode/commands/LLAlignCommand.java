package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.util.PIDConstants.LLAlignKD;
import static org.firstinspires.ftc.teamcode.util.PIDConstants.LLAlignKI;
import static org.firstinspires.ftc.teamcode.util.PIDConstants.LLAlignKP;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class LLAlignCommand extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final double setpoint = 0;
    private final double maxYawSpeed = 1; // max rotation speed
//    double yaw;
    long lastTime = System.nanoTime();
    double output;
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    PIDController PID = new PIDController(LLAlignKP, LLAlignKI, LLAlignKD, setpoint, maxYawSpeed); // Initialize pid controller
    private final LimelightSubsystem ll;
    double error = 0;
    boolean target = false;

    public LLAlignCommand(MecanumDriveSubsystem drive, LimelightSubsystem ll, Gamepad gamepad, Telemetry telemetry) {
        this.drive = drive;
        this.ll = ll;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        telemetry.addData("excecuting LLalignCommand", "true");
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
        } else {
            LLResult result = ll.getLatestResult();
            telemetry.addData("hasTarget", "false");
            if (result == null) telemetry.addData("resultNull", "true");
            assert result != null;
            if (!result.isValid()) telemetry.addData("resultValid", "false");
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
