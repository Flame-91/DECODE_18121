package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.dashboard.config.Config;


@Config
public class ConfigConstants {
    public static double LLAlignKP = 0.1;
    public static double LLAlignKI = 0.01;
    public static double LLAlignKD = 0.00001;

    public static double pivotKP = 0.1;
    public static double pivotKI = 0.01;
    public static double pivotKD = 0.0001;
    public static final double encoderTicksM = 5;  //Not necessary to be stored as a variable as it is a constant only used in this class
    public static final double encoderTicksB = 5;
}
