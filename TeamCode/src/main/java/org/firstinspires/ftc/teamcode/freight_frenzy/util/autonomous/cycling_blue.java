package org.firstinspires.ftc.teamcode.freight_frenzy.util.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group="blue")
@Disabled
public class cycling_blue extends LinearOpMode {
    AutoFlow auto = new AutoFlow(this, AutoFlow.ALLIANCE.BLUE, AutoFlow.StartPos.BARRIER, AutoFlow.Auto.CYCLING);

    @Override
    public void runOpMode() {
        auto.init();

        waitForStart();

        auto.run();
    }
}
