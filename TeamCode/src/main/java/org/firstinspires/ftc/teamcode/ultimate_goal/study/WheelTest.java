package org.firstinspires.ftc.teamcode.ultimate_goal.study;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "Cobalt")
public class WheelTest extends LinearOpMode {
    private DcMotorEx fl = null;
    private DcMotorEx fr = null;
    private DcMotorEx bl = null;
    private DcMotorEx br = null;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    MultipleTelemetry multipleTelemetry;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.FORWARD);

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        String state = "";

        waitForStart();
        while (opModeIsActive()) {
            fl.setPower(gamepad1.left_stick_x);
            fr.setPower(gamepad1.right_stick_x);

            bl.setPower(-gamepad1.left_stick_y);
            br.setPower(-gamepad1.right_stick_y);
            if (gamepad1.a) {
                fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                state = "USING ENCODER";
            }
            if (gamepad1.b) {
                fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                state = "WITHOUT ENCODER";
            }

            multipleTelemetry.addData("front left:", String.format("\tPos: %d \n\tCurrent: %f", fl.getCurrentPosition(), fl.getCurrent(CurrentUnit.AMPS)));
            multipleTelemetry.addData("front right:", String.format("\tPos: %d \n\tCurrent: %f", fr.getCurrentPosition(), fr.getCurrent(CurrentUnit.AMPS)));
            multipleTelemetry.addData("back left:", String.format("\tPos: %d \n\tCurrent: %f", bl.getCurrentPosition(), bl.getCurrent(CurrentUnit.AMPS)));
            multipleTelemetry.addData("back right:", String.format("\tPos: %d \n\tCurrent: %f", br.getCurrentPosition(), br.getCurrent(CurrentUnit.AMPS)));
            multipleTelemetry.addData("state: ", state);
            multipleTelemetry.update();
        }
    }
}
