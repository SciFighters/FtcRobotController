package org.firstinspires.ftc.teamcode.centerstage.Autonomous.Paths.BlueAlliance.Backdrop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@Autonomous(group = "BLUE", name = "Blue: Backdrop - ParkOnly P-Near")
public class BlueBackdropkParkOnlyNear extends Robot {
    AutoFlow auto;

    @Override
    public void initRobot() {
        auto = addComponent(AutoFlow.class, new AutoFlow(this, AutoFlow.Alliance.BLUE,
                AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.PARK, AutoFlow.ParkLocation.RIGHT));
    }

    @Override
    public void startRobot() {
        auto.runPath();
    }
}
