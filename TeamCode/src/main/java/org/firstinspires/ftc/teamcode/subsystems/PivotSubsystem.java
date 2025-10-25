package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PivotSubsystem extends SubsystemBase {
    private final DcMotor leftPivotMotor;
    private final DcMotor rightPivotMotor;
    private final JoinedTelemetry telemetry;
    double offsetAngleFromLimelightToPivot = -5;
    public PivotSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry) {
        leftPivotMotor = hardwareMap.get(DcMotor.class, "leftPivotMotor");
        rightPivotMotor = hardwareMap.get(DcMotor.class, "rightPivotMotor");
        leftPivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.telemetry = telemetry;

        register();
    }

    @Override
    public void periodic() {
        telemetry.addData("pivotPower", getPivotPower());
        telemetry.addData("currentPivotPosition", getCurrentPivotPosition());
        telemetry.addData("targetPivotPosition", getTargetPivotPosition());
    }

    public void setPivotPower(double power) {
        leftPivotMotor.setPower(power);
        rightPivotMotor.setPower(power);
    }

    public void setPivotTargetPosition(int position) {
        leftPivotMotor.setTargetPosition(position - convertPivotAngleToTicks(offsetAngleFromLimelightToPivot));
        rightPivotMotor.setTargetPosition(position - convertPivotAngleToTicks(offsetAngleFromLimelightToPivot));
    }

    public double convertPivotTicksToAngle(double ticks) {
        return ticks * (360/537.7); // 384.5 is PPR of motor (resolution of encoder) and this value depends on which motor we r using but the PPR is available on GoBilda.com
    }
    public void movePivotWithoutEncoder(double power) {
        leftPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPivotMotor.setPower(power);
        rightPivotMotor.setPower(power);
    }
    public void resetPivotEncoder() { // Should only be called if pivot is at rest DOWN
        leftPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public int convertPivotAngleToTicks(double angle) {
        return (int) (angle * (537.7/360));
    }

    public double getCurrentPivotPosition() { return leftPivotMotor.getCurrentPosition()  - convertPivotAngleToTicks(offsetAngleFromLimelightToPivot); }
    public double getTargetPivotPosition() { return leftPivotMotor.getTargetPosition()  - convertPivotAngleToTicks(offsetAngleFromLimelightToPivot); }
    public double getPivotPower() { return leftPivotMotor.getPower(); }
}
