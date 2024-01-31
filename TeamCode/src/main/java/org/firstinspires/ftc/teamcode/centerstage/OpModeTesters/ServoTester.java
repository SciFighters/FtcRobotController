package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Input;

@TeleOp
@Disabled
public class ServoTester extends LinearOpMode {
    Servo servo1, servo2;
    double min = 0.325, max;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        Input.init(this, gamepad1, gamepad2);
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
        servo1 = hardwareMap.get(Servo.class, "backClawServo");
        servo2 = hardwareMap.get(Servo.class, "frontClawServo");
        servo1.scaleRange(min, 1);
        servo2.scaleRange(min, 1);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double increment = 0, increment2 = 0;
            if (gamepad1.dpad_up) {
                servo1.setPosition(servo1.MIN_POSITION);
            }
            if (gamepad1.dpad_down) {
                servo1.setPosition(servo1.MAX_POSITION);
            }
            if (gamepad1.dpad_left) {
                servo2.setPosition(servo2.MIN_POSITION);
            }
            if (gamepad1.dpad_right) {
                servo2.setPosition(servo2.MAX_POSITION);
            }
            multipleTelemetry.addData("Servo1 pos", servo1.getPosition());
            multipleTelemetry.addData("Servo2 pos", servo2.getPosition());
            multipleTelemetry.addData("MAX: ", max);
            multipleTelemetry.addData("MIN: ", min);
            multipleTelemetry.update();
        }

    }
}