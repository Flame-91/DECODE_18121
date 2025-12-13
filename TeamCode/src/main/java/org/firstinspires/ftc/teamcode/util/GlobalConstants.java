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
    public static double pivotKF = 0.15;
    public static double pivotShootingOffset = 0.42545;
    public static double pivotTicksAtNinety = 67; // MAKE SURE to subtract offsets (which is the angle of when pivot is down)
    public static double pivotTicksAtFortyFive = 83; // Use telemetry to print encoder ticks, and then use angle to get linear equation (p value)

    public static double flywheelKP = 1;
    public static double flywheelKI = 0.1;
    public static double flywheelKD = 0.001;

    public static double flywheelMotorPower = 0.3;
    public static double flywheelMotorRuntime = 2.75;
    public static double flywheelServoRuntime = 0.2;
    public static double flywheelServoBreaktime1 = 0.75;
    public static double flywheelServoBreaktime2 = 1.25;
}
