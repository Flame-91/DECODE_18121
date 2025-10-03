package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase {
    private DcMotor leftMotor, rightMotor;

    public TankDriveSubsystem(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double x, double y, double rotation) {
        // In tank drive, x is ignored. y controls forward/backward, rotation controls turning.
        double leftPower = y + rotation;
        double rightPower = y - rotation;

        // Normalize powers to be within -1 to 1
        double max = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower /= max;
        rightPower /= max;

        // Set motor powers
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }
}
