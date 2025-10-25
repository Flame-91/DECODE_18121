package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelSubsystem extends SubsystemBase {
    private final DcMotor leftFlywheelMotor;
    private final DcMotor rightFlywheelMotor;
    private final JoinedTelemetry telemetry;
    public FlywheelSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry) {
        this.leftFlywheelMotor = hardwareMap.get(DcMotor.class, "leftFlywheelMotor");
        this.rightFlywheelMotor = hardwareMap.get(DcMotor.class, "rightFlywheelMotor");

        leftFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("flywheelPower", getLeftFlywheelMotorPower());
        telemetry.addData("servoPower", getRightFlywheelMotorPower());
    }

    public void setFlywheelMotorPower(double power) {
        leftFlywheelMotor.setPower(power);
        rightFlywheelMotor.setPower(power);
    }

    public double getLeftFlywheelMotorPower() { return leftFlywheelMotor.getPower(); }
    public double getRightFlywheelMotorPower() { return rightFlywheelMotor.getPower(); }
}
