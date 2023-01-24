package org.firstinspires.ftc.teamcode.power_play.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "BLUE")
//@Disabled
public class Left_High extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoFlow auto = new AutoFlow(this, AutoFlow.StartPos.LEFT, AutoFlow.Auto.FULL); //bacl

        auto.init();

        waitForStart();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.startTime();
        auto.placeFirstHighCone();
        sleep(200);
        //auto.placeCone(auto.highJunction, 5);
        auto.goToParkingPosition();
        //auto.placeCones();
        telemetry.addData("Time ", timer.time());
        telemetry.update();
        sleep(20_000);
    }
}
