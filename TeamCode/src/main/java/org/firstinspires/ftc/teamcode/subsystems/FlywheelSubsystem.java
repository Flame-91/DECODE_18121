package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelSubsystem extends SubsystemBase {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotor flywheelMotor;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    double servoPower;
    double flywheelMotorPower;
    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;
    }

    @Override
    public void periodic() {
        telemetry.addData("Flywheel Power", flywheelMotorPower);
        telemetryPacket.put("Flywheel Power", flywheelMotorPower);
        dashboard.sendTelemetryPacket(telemetryPacket);

        telemetry.addData("Servo Power", servoPower);
        telemetryPacket.put("Servo Power", servoPower);
        dashboard.sendTelemetryPacket(telemetryPacket);
    }

    public void setFlywheelMotor(double power) {
        if (power < -1 || power > 1) return;
        flywheelMotor.setPower(power);
        flywheelMotorPower = power;
    }

    public void setServosPower(double power) {
        if (power < -1 || power > 1) return;
        rightServo.setPower(power);
        leftServo.setPower(power);

        servoPower = power;
    }
}
