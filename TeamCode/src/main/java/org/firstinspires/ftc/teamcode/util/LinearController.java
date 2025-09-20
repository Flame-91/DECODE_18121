package org.firstinspires.ftc.teamcode.util;

import kotlin.OverloadResolutionByLambdaReturnType;

public class LinearController {
    private final double slope;
    private final double yInt;
    private final double lowerClamp;
    private final double higherClamp;
//    private double setPoint;
    public LinearController(double slope, double yInt, double lowerClamp, double higherClamp) {
        this.slope = slope;
        this.yInt = yInt;
        this.lowerClamp = lowerClamp;
        this.higherClamp = higherClamp;
    }

//    public void setSetPoint(double setPoint) {
//        this.setPoint = setPoint;
//    }

    public double calculate(double error) {
        double answer;
        answer = -((error*slope)+yInt);
        if (answer > higherClamp) answer = higherClamp;
        if (answer < lowerClamp) answer = lowerClamp;
        return answer;
    }
}
