package org.firstinspires.ftc.teamcode.power_play;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.power_play.util.Lift;
@TeleOp
public class TickTest extends LinearOpMode {

    Lift lift = new Lift();
    ElapsedTime timer = new ElapsedTime();

    public void initiate() {
        lift.init(hardwareMap);

    }

    @Override
    public void runOpMode() {
        telemetry.addData("Initializing", "...");
        telemetry.update();
        initiate();

        waitForStart();
            while (opModeIsActive()) {

                telemetry.addData("Ticks Position", lift.getPos());
                telemetry.addData("Relative Position", lift.getRelativePos());
                telemetry.update();
        }
    }
}
