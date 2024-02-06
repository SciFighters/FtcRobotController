package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.BlueAlliance.PixelStack;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.DuckLine;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "BLUE", name = "Blue: Pixel Stack - Full")
public class BluePixelStackFull extends Robot {
    AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.BLUE,
            AutoFlow.StartPos.PIXEL_STACK, AutoFlow.Auto.FULL);

    @Override
    public void initRobot() {
        auto.init();
    }

    @Override
    public void startRobot() {
        auto.pixelStackSideBlue();
    }
}
