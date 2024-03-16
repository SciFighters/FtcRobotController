package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.RedAlliance.Backdrop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "RED", name = "Red: Backdrop - ParkOnly P-Far")
public class RedBackdropkParkOnlyFar extends Robot {
    AutoFlow auto;

    @Override
    public void initRobot() {
        auto = addComponent(AutoFlow.class, new AutoFlow(this, AutoFlow.Alliance.RED,
                AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.PARK, AutoFlow.ParkLocation.FAR));
    }

    @Override
    public void startRobot() {
        auto.runPath();
    }
}
