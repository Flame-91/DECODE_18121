package org.firstinspires.ftc.teamcode.kitbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class KitbotTeleOp extends OpMode {
    OmniDriveSubsystem omniDriveSubsystem;

    @Override
    public void init() {
        omniDriveSubsystem = new OmniDriveSubsystem(hardwareMap);
    }
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;
        omniDriveSubsystem.OmniDrive(y, rotation);

    }
}