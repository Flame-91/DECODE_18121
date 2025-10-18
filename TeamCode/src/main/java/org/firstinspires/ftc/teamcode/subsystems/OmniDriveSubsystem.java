package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OmniDriveSubsystemOG extends SubsystemBase {
    private final DcMotor left, right;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    public OmniDriveSubsystemOG(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void OmniDrive() {
        double y = -gamepad.left_stick_y;
        double rotation = gamepad.right_stick_x;

        double leftPower = y + rotation;
        double rightPower = y - rotation;

        double max = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));

        leftPower /= max;
        rightPower /= max;

        left.setPower(leftPower);
        right.setPower(rightPower);

        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);
        telemetry.update();
    }
}
