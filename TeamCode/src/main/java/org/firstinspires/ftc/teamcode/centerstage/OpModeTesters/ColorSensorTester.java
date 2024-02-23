package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;


@TeleOp(group = "TESTER")
public class ColorSensorTester extends Robot {
    ColorSensor armSensor;

    @Override
    public void initRobot() {
        armSensor = hardwareMap.get(ColorSensor.class, "armColorSensor");
    }

    @Override
    public void startRobot() {

    }

    @Override
    public void updateLoop() {
        telemetry.addData("Color sensor rgb", String.format("%d, %d, %d", armSensor.red(), armSensor.green(), armSensor.blue()));
        telemetry.update();
    }
}