package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous(name = "CompAutoBlueGoal")
public class CompAutoBlueGoal extends OpMode {
    private enum ScoreState {
        REV,
        RUN1,
        BREAK1,
        RUN2,
        BREAK2,
        RUN3,
        DELAY,
        MOVE,
        DONE
    }
    ScoreState scoreState = ScoreState.REV;



}