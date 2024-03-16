package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.RedAlliance.Backdrop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "RED", name = "Red: Backdrop - Full P-Far")
public class RedBackdropkFullFar extends Robot {
    AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.RED,
            AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.FULL, AutoFlow.ParkLocation.FAR);

    @Override
    public void initRobot() {
        auto.init();
    }

    @Override
    public void startRobot() {
        auto.runPath();
    }
}
