package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Tars extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoFlow auto = new AutoFlow(this, AutoFlow.Alliance.BLUE,
                AutoFlow.StartPos.PIXEL_STACK, AutoFlow.Auto.FULL);

        auto.init();

        waitForStart();

        auto.teaching();
    }
}