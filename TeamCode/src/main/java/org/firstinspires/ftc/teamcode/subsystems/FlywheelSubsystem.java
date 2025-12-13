package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelSubsystem extends SubsystemBase {
//    private final DcMotor leftFlywheelMotor;
    private final DcMotor rightFlywheelMotor;
    private final CRServo leftFlywheelServo;
    private final CRServo rightFlywheelServo;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
//        this.leftFlywheelMotor = hardwareMap.get(DcMotor.class, "leftFlywheelMotor");
        this.rightFlywheelMotor = hardwareMap.get(DcMotor.class, "rightFlywheelMotor");
        this.leftFlywheelServo = hardwareMap.get(CRServo.class, "leftFlywheelServo");
        this.rightFlywheelServo = hardwareMap.get(CRServo.class, "rightFlywheelServo");

//        leftFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheelServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFlywheelServo.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("flywheelPower", getLeftFlywheelMotorPower());
        telemetry.addData("servoPower", getRightFlywheelServoPower());

        telemetryPacket.put("flywheelPower", getLeftFlywheelMotorPower());
        telemetryPacket.put("servoPower", getRightFlywheelServoPower());
    }

    public void setFlywheelMotorPower(double power) {
//        leftFlywheelMotor.setPower(power);
        rightFlywheelMotor.setPower(power);
    }

    public void setFlywheelServoPower(double power) {
        leftFlywheelServo.setPower(power);
        rightFlywheelServo.setPower(power);
    }

    public double getLeftFlywheelMotorPower() { return rightFlywheelMotor.getPower(); }
    public double getRightFlywheelMotorPower() { return rightFlywheelMotor.getPower(); }
    public double getLeftFlywheelServoPower() { return leftFlywheelServo.getPower(); }
    public double getRightFlywheelServoPower() { return rightFlywheelServo.getPower(); }
}