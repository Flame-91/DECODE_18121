package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;


@Configurable
@Config
public class GlobalConstants {
    public static double LLAlignKP = 0.1;
    public static double LLAlignKI = 0.01;
    public static double LLAlignKD = 0.00001;

    public static double pivotKP = 0.05;
    public static double pivotKI = 0.005;
    public static double pivotKD = 0.0001;

    public static double flywheelMotorPower = 1;
    public static double flywheelMotorRuntime = 2.75;
    public static double flywheelServoRuntime = 0.2;
    public static double flywheelServoBreaktime1 = 0.75;
    public static double flywheelServoBreaktime2 = 1.25;

    public static double imuOffset = 0;
}
