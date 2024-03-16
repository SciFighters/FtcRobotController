package org.firstinspires.ftc.teamcode.freight_frenzy.study;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class motor_test extends LinearOpMode {

    volatile private DcMotorEx collector = null;

    private void Init(HardwareMap map) {
        collector = map.get(DcMotorEx.class, "collector");
        collector.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        collector.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Init(hardwareMap);
        waitForStart();
        if(gamepad1.dpad_down) {
            collector.setPower(1);
            telemetry.addData("pressing","NEAR");
        } else {
            collector.setPower(0);
            telemetry.addData("not pressing","NEAR");
        }
        telemetry.update();
    }
}
