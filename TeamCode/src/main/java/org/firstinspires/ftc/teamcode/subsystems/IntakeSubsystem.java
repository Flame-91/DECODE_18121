package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    CRServo flywheel;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(CRServo.class, "flywheel");
    }

    public void setFlywheelPower(double power) {
        flywheel.setPower(power);
    }
    public double getFlywheelPower() {
        return flywheel.getPower();
    }
}
