package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.BlueAlliance.PixelStack;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "BLUE", name = "Blue: Pixel Stack - Cycle P-Near")
@Disabled
public class BluePixelStackCycleNear extends Robot {
    AutoFlow auto;

    @Override
    public void initRobot() {
        auto = addComponent(AutoFlow.class, new AutoFlow(this, AutoFlow.Alliance.BLUE,
                AutoFlow.StartPos.PIXEL_STACK, AutoFlow.Auto.CYCLING, AutoFlow.ParkLocation.NEAR));
    }

    @Override
    public void startRobot() {
        auto.runPath();
    }
}
