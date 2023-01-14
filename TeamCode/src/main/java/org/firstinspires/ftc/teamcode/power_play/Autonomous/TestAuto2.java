package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(group = "BLUE")
//@Disabled
public class TestAuto2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoFlow auto = new AutoFlow(this, AutoFlow.StartPos.LEFT, AutoFlow.Auto.FULL); //bacl

        auto.init();

        waitForStart();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();
        auto.test();
        //auto.gotoParkingPosition(AutoFlow.ParkingPosition.three);
        telemetry.addData("Time ", timer.time());
        telemetry.update();
        sleep(20_000);
    }
}
