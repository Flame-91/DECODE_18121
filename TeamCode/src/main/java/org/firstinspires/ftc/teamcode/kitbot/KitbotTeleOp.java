package org.firstinspires.ftc.teamcode.kitbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class KitbotTeleOp extends OpMode {
    OmniDriveSubsystem omniDriveSubsystem;
    FlyWheelSubsystem flyWheelSubsystem;
    @Override
    public void init() {
        omniDriveSubsystem = new OmniDriveSubsystem(hardwareMap);
        flyWheelSubsystem = new FlyWheelSubsystem(hardwareMap);
    }
    @Override
    public void loop() {
        omniDriveSubsystem.OmniDrive();
        flyWheelSubsystem.FlyWheelLaunch();
    }
}