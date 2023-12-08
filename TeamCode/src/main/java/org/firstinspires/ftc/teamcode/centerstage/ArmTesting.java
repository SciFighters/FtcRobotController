package org.firstinspires.ftc.teamcode.centerstage;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;

@TeleOp
public class ArmTesting extends LinearOpMode {
    //    DcMotorEx lift1, lift2;
//    Servo grabberLeft, grabberRight;
    Arm arm;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
//        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
//        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
//        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
//
//        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
//        grabberRight = hardwareMap.get(Servo.class, "grabberRight");
        arm = new Arm(this, telemetry);
        Thread armThread = new Thread(arm);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            double power = (gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y));
            if (!arm.isOverCurrentLimit()) {
                arm.setPower(power);
            }
            dashboardTelemetry.addData("LIFT", arm.isOverCurrentLimit());
            dashboardTelemetry.addData("LIFT Current", arm.getCurrent());
            dashboardTelemetry.addData("Lift pos", arm.getPos());
            dashboardTelemetry.update();
//            lift1.setPower(power);
//            lift2.setPower(power);
//
//            grabberLeft.setPosition(gamepad2.right_stick_x);
//            grabberRight.setPosition(gamepad2.right_stick_y);

//            telemetry.addData("Left Lift", lift1.getCurrentPosition());
//            telemetry.addData("Right Lift", lift2.getCurrentPosition());
        }
        armThread.interrupt();
    }
}
