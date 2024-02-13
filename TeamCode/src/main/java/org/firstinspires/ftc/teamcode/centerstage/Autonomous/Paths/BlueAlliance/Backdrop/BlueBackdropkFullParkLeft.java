package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.BlueAlliance.Backdrop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "BLUE", name = "Blue: Backdrop - Full P-Left")
public class BlueBackdropkFullParkLeft extends Robot {
    AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.BLUE,
            AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.FULL, AutoFlow.ParkLocation.LEFT);

    @Override
    public void initRobot() {
        auto.init();
    }

    @Override
    public void startRobot() {
        auto.run();
    }
}
