package org.firstinspires.ftc.teamcode.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmTesting extends LinearOpMode {
    DcMotorEx lift1, lift2;
    Servo grabberLeft, grabberRight;

    @Override
    public void runOpMode() throws InterruptedException {

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");

        while (opModeIsActive() && !isStopRequested()) {
            double power = (gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y)) / 2;
            lift1.setPower(power);
            lift2.setPower(power);

            grabberLeft.setPosition(gamepad2.right_stick_x);
            grabberRight.setPosition(gamepad2.right_stick_y);
        }
    }
}
