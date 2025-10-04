package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotor flywheelMotor;
    private final ElapsedTime timer = new ElapsedTime();
    public FlywheelSubsystem( HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void score() {
        leftServo.setPower(1);
        rightServo.setPower(1);
        flywheelMotor.setPower(1);
        timer.reset();
        while (timer.seconds() < 0.5) {}
        leftServo.setPower(0);
        rightServo.setPower(0);

        timer.reset();

        while (timer.seconds() < 0.3) {}

        flywheelMotor.setPower(0);
    }
}
