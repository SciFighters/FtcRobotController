package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.RedAlliance.PixelStack;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Disabled
@Autonomous(group = "RED", name = "Red: Pixel Stack - Cycle P-Near")
public class RedPixelStackCycleNear extends Robot {
    AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.RED,
            AutoFlow.StartPos.PIXEL_STACK, AutoFlow.Auto.CYCLING, AutoFlow.ParkLocation.NEAR);

    @Override
    public void initRobot() {
        auto.init();
    }

    @Override
    public void startRobot() {
        auto.runPath();
    }
}
