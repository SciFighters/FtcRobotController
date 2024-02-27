package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@TeleOp(group = "TESTER")
public class ArmTesting extends Robot {
    Arm arm;
    FtcDashboard dashboard;
    MultipleTelemetry multipleTelemetry;
    Telemetry dashboardTelemetry;

    @Override
    public void initRobot() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
        arm = addComponent(Arm.class);
    }

    @Override
    public void updateLoop() {
        double power = -gamepad2.left_stick_y;
        arm.setMotorsPower(power);
        multipleTelemetry.addData("LIFT", arm.isOverCurrentLimit());
        multipleTelemetry.addData("LIFT Current", arm.getCurrent());
        multipleTelemetry.addData("Lift pos", arm.pos());
        multipleTelemetry.addData("Lift 2 pos", arm.getCurrent2());
        multipleTelemetry.addData("Lift 2 pos", arm.pos2());
        multipleTelemetry.addData("Power", power);
        multipleTelemetry.update();
    }


    @Override
    public void startRobot() {

    }
}
