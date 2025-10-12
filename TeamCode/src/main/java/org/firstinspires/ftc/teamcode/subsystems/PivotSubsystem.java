package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.ConfigConstants.encoderTicksB;
import static org.firstinspires.ftc.teamcode.util.ConfigConstants.encoderTicksM;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem extends SubsystemBase {
    private final DcMotor leftPivotMotor;
    private final DcMotor rightPivotMotor;
    private final Telemetry telemetry;
    private final TelemetryPacket telemetryPacket;
    public PivotSubsystem(HardwareMap hardwareMap, Telemetry telemetry, TelemetryPacket telemetryPacket) {
        leftPivotMotor = hardwareMap.get(DcMotor.class, "leftPivotMotor");
        rightPivotMotor = hardwareMap.get(DcMotor.class, "rightPivotMotor");
        leftPivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.telemetry = telemetry;
        this.telemetryPacket = telemetryPacket;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("pivotPower", getPivotPower());
        telemetry.addData("currentPivotPosition", getCurrentPivotPosition());
        telemetry.addData("targetPivotPosition", getTargetPivotPosition());

        telemetryPacket.put("pivotPower", getPivotPower());
        telemetryPacket.put("currentPivotPosition", getCurrentPivotPosition());
        telemetryPacket.put("targetPivotPosition", getTargetPivotPosition());
    }

    public void setPivotPower(double power) {
        leftPivotMotor.setPower(power);
        rightPivotMotor.setPower(power);
    }
    public void setPivotPosition(int position) {
        leftPivotMotor.setTargetPosition(position);
        rightPivotMotor.setTargetPosition(position);
    }
    public double convertPivotTicksToAngle(double ticks) {
        return (ticks - encoderTicksB)/encoderTicksM;
    }

    public int convertPivotAngleToTicks(double angle) {
        return (int) Math.floor((angle * encoderTicksM) + encoderTicksB);
    }

    public double getCurrentPivotPosition() { return leftPivotMotor.getCurrentPosition(); }
    public double getTargetPivotPosition() { return leftPivotMotor.getTargetPosition(); }
    public double getPivotPower() { return leftPivotMotor.getPower(); }
}
