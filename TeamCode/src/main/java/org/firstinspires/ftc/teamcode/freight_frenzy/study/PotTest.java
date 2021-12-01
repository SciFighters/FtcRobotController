package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "PotTest")
public class PotTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput pot = hardwareMap.get(AnalogInput.class, "pot");

        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();

        waitForStart();

        pot.getVoltage();

        while (opModeIsActive()) {
            telemetry.addData("pot:", pot.getVoltage());
            telemetry.update();
        }
    }
}
