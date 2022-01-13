package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.HandRailClass;

@TeleOp(name = "PotTest")
public class PotTest extends LinearOpMode {
    DcMotorEx hand = null;

    final double handRang = 5000;

    private int patent(double pv){
        int tick = (int)(handRang * (1- pv / 3.33));
        return tick;
    }

    @Override
    public void runOpMode() {
        AnalogInput pot = hardwareMap.get(AnalogInput.class, "potentiometer");
        hand = hardwareMap.get(DcMotorEx.class, "hand");
        hand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();

        waitForStart();

        pot.getVoltage();

        while (opModeIsActive()) {

            telemetry.addData("potentiometer V", pot.getVoltage());
            telemetry.addData("potentiometer %",pot.getVoltage() / pot.getMaxVoltage()* 100);
            telemetry.addData("ticks: ", hand.getCurrentPosition());
            telemetry.addData("current offset", patent(pot.getVoltage()));
            telemetry.addData("current diff", hand.getCurrentPosition() - patent(pot.getVoltage()));

            telemetry.update();

        }
    }
}
