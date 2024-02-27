package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.RedAlliance.Backdrop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "RED", name = "Red: Backdrop - Cycle P-Far")
public class RedBackdropkCyclParkFar extends Robot {
    AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.RED,
            AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.CYCLING, AutoFlow.ParkLocation.LEFT);

    @Override
    public void initRobot() {
        auto.init();
    }

    @Override
    public void startRobot() {
        auto.runPath();
    }
}
