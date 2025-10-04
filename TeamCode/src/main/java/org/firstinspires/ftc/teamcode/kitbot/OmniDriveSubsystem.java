package org.firstinspires.ftc.teamcode.kitbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class OmniDriveSubsystem extends SubsystemBase {
    DcMotor left, right;
    public OmniDriveSubsystem(HardwareMap hardwareMap) {
        //must add motor names in config through driver station
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Put upside down and test if necessary
//        left.setDirection(DcMotor.Direction.REVERSE);
//        right.setDirection(DcMotor.Direction.REVERSE);
    }
    public void OmniDrive(double y, double rotation) {
        double leftMotorPower = y + rotation;
        double rightMotorPower = y - rotation;

        double max = Math.max(1, Math.max(Math.abs(leftMotorPower), Math.abs(rightMotorPower)));

        leftMotorPower /= max;
        rightMotorPower /= max;
        left.setPower(leftMotorPower);
        right.setPower(rightMotorPower);
    }
}
