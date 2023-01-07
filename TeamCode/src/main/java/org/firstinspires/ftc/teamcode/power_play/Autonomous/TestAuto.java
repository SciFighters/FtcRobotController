package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "BLUE")
//@Disabled
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoFlow auto = new AutoFlow(this, AutoFlow.ALLIANCE.BLUE, AutoFlow.StartPos.back, AutoFlow.Auto.FULL);

        auto.init();

        waitForStart();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();
        auto.placeFirstCone();
        auto.placeCones();
        telemetry.addData("Time ", timer.time());
        telemetry.update();
        sleep(20_000);
    }
}
