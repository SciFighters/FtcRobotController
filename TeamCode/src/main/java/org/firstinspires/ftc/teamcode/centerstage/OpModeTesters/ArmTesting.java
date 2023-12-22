package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;

@TeleOp(group = "CENTERSTAGE")
public class ArmTesting extends LinearOpMode {
    Arm arm;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        MultipleTelemetry multipleTelemetry;
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
        arm = new Arm(this, telemetry);
        Thread armThread = new Thread(arm);
        waitForStart();
        armThread.start();
        while (opModeIsActive() && !isStopRequested()) {
            double power = gamepad2.left_stick_y;
            arm.setMotorsPower(power);
            multipleTelemetry.addData("LIFT", arm.isOverCurrentLimit());
            multipleTelemetry.addData("LIFT Current", arm.getCurrent());
            multipleTelemetry.addData("Lift pos", arm.getPos());
            multipleTelemetry.addData("Power", power);
            multipleTelemetry.update();
        }

        armThread.interrupt();
    }
}
