package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "BLUE", name = "TEST R")
public class TestRed extends Robot {
    AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.RED, AutoFlow.StartPos.PIXEL_STACK, AutoFlow.Auto.FULL);

    @Override
    public void initRobot() {
        auto.init();
    }

    @Override
    public void startRobot() {
        auto.test();
    }
}
