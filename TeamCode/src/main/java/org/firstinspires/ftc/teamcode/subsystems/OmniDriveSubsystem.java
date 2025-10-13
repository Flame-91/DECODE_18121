package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class OmniDriveSubsystem extends SubsystemBase {
    private final DcMotor left, right;
    private final Gamepad gamepad;
    public OmniDriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad) {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        this.gamepad = gamepad;
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.REVERSE);
    }
    public void OmniDrive(double forward, double rotation) {
        double leftMotorPower = forward + rotation;
        double rightMotorPower = forward - rotation;

        double max = Math.max(1, Math.max(Math.abs(leftMotorPower), Math.abs(rightMotorPower)));

        leftMotorPower /= max;
        rightMotorPower /= max;

        left.setPower(leftMotorPower);
        right.setPower(rightMotorPower);
    }
}