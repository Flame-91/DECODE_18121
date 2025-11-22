package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlyWheelSubsystem extends SubsystemBase {
    private final DcMotor flywheelMotor;
    private final CRServo rightFlyWheel, leftFlyWheel;
    private final Telemetry telemetry;
    public FlyWheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        rightFlyWheel = hardwareMap.get(CRServo.class, "rightFlyWheel");
        leftFlyWheel = hardwareMap.get(CRServo.class, "leftFlyWheel");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetry;
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("Servo Power", getServoPower());
        telemetry.addData("Motor Power", getMotorPower());
    }

    public void runFlywheelServos(double power) {
        rightFlyWheel.setPower(power);
        leftFlyWheel.setPower(power);
    }

    public void runFlywheel(double power) {
        flywheelMotor.setPower(power);
    }

    public double getServoPower() {
        return rightFlyWheel.getPower();
    }

    public double getMotorPower() {
        return flywheelMotor.getPower();
    }
}