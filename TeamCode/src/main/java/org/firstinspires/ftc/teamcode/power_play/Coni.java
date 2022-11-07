package org.firstinspires.ftc.teamcode.power_play;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.power_play.util.Lift;

public class Coni extends LinearOpMode {

    Lift lift = new Lift();
    ElapsedTime timer = new ElapsedTime();

    public void initiate() {
        lift.init(hardwareMap);

    }

    @Override
    public void runOpMode() {
        telemetry.addData("Mode", "Initiating");
        telemetry.update();

        this.initiate();

        telemetry.addData("Mode", "Ready for start!");

        waitForStart();

        lift.setPower(gamepad1.left_stick_y * 0.4 + 0.4);

    }
}
