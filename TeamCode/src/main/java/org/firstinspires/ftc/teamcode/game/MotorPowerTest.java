package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@Autonomous(name = "MotorPowerTest")
public class MotorPowerTest extends OpMode {
    MecanumDriveSubsystem mecanumDriveSubsystem;
    double frontLeftPower = -0.225, frontRightPower = -0.4, backLeftPower = -0.225, backRightPower = -0.4;
    double duration = 3.0;
    ElapsedTime elapsedTime;
    @Override
    public void init() {
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap, telemetry);
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        if (elapsedTime.seconds() <= duration) {
            mecanumDriveSubsystem.drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        } else {
            mecanumDriveSubsystem.drive(0, 0, 0, 0);
            requestOpModeStop();
        }
    }

    @Override
    public void stop() {
        if (mecanumDriveSubsystem != null) {
            mecanumDriveSubsystem.drive(0, 0, 0, 0);
        }
    }
}
