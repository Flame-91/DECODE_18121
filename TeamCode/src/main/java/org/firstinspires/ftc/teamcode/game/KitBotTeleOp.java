package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlyWheelSubsystemOG;
import org.firstinspires.ftc.teamcode.subsystems.OmniDriveSubsystemOG;

@TeleOp (name = "Kit Bot TeleOp")
public class KitBotTeleOp extends OpMode {
    FlyWheelSubsystemOG flyWheelSubsystem;
    OmniDriveSubsystemOG omniDriveSubsystem;

    @Override
    public void init() {
        flyWheelSubsystem = new FlyWheelSubsystemOG(hardwareMap, gamepad1, telemetry);
        omniDriveSubsystem = new OmniDriveSubsystemOG(hardwareMap, gamepad1, telemetry);
    }

    @Override
    public void loop() {
        flyWheelSubsystem.FlyWheelLaunch();
        omniDriveSubsystem.OmniDrive();
    }
}
