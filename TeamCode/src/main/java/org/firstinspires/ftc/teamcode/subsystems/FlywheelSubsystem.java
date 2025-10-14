package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelSubsystem extends SubsystemBase {
    private final DcMotor leftFlywheelMotor;
    private final DcMotor rightFlywheelMotor;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        this.leftFlywheelMotor = hardwareMap.get(DcMotor.class, "leftFlywheelMotor");
        this.rightFlywheelMotor = hardwareMap.get(DcMotor.class, "rightFlywheelMotor");
        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("flywheelPower", getLeftFlywheelMotorPower());
        telemetry.addData("servoPower", getRightFlywheelMotorPower());

        telemetryPacket.put("flywheelPower", getLeftFlywheelMotorPower());
        telemetryPacket.put("servoPower", getRightFlywheelMotorPower());
    }

    public void setFlywheelMotorPower(double power) {
        leftFlywheelMotor.setPower(power);
        rightFlywheelMotor.setPower(power);
    }

    public double getLeftFlywheelMotorPower() { return leftFlywheelMotor.getPower(); }
    public double getRightFlywheelMotorPower() { return rightFlywheelMotor.getPower(); }
}
