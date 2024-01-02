package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Cheese extends LinearOpMode {
    DcMotor motor1 = null;
    DcMotor motor2 = null;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "left");
        motor2 = hardwareMap.get(DcMotor.class, "right");
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double y = gamepad1.left_stick_y;
            motor1.setPower(y);
            motor2.setPower(y);
        }

    }
}