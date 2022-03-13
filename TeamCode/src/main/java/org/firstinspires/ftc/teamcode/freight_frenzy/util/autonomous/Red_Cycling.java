package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group="RED")
//@Disabled
public class Red_Cycling extends LinearOpMode {
    AutoFlow auto = new AutoFlow(this, AutoFlow.ALLIANCE.RED, AutoFlow.StartPos.BARRIER, AutoFlow.Auto.CYCLING);

    @Override
    public void runOpMode() {
        auto.init();

        waitForStart();

        auto.run();
    }
}
