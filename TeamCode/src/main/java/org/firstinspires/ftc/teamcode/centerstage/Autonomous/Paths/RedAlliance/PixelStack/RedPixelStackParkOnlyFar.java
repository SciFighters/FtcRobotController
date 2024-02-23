package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.RedAlliance.PixelStack;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "RED", name = "Red: PixelStack - ParkOnly P-Far")
public class RedPixelStackParkOnlyFar extends Robot {
    AutoFlow auto;

    @Override
    public void initRobot() {
        auto = addComponent(AutoFlow.class, new AutoFlow(this, AutoFlow.Alliance.RED,
                AutoFlow.StartPos.PIXEL_STACK, AutoFlow.Auto.PARK, AutoFlow.ParkLocation.LEFT));
    }

    @Override
    public void startRobot() {
        auto.runPath();
    }
}
