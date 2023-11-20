package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "BLUE")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.BLUE,
                AutoFlow.StartPos.BACKSTAGE, AutoFlow.Auto.FULL);

        auto.init();

        waitForStart();

        auto.test();
    }
}
