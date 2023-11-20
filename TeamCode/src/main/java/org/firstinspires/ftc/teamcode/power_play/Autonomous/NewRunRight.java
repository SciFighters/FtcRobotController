package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "BLUE")
@Disabled
public class NewRunRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoFlow auto = new AutoFlow(this, AutoFlow.StartPos.RIGHT, AutoFlow.Auto.FULL); //bacl

        auto.init();

        waitForStart();

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();

        auto.newRun(1);

        telemetry.addData("Time ", timer.time());
        telemetry.update();
//        sleep(20_000);
    }
}
