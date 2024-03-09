package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;


@TeleOp(group = "TESTER")
public class ColorSensorTester extends Robot {
    ColorSensor farPixelColorSensor, nearPixelColorSensor;
    boolean pixelHere;

    @Override
    public void initRobot() {
        farPixelColorSensor = hardwareMap.get(ColorSensor.class, "farPixelColorSensor");
        nearPixelColorSensor = hardwareMap.get(ColorSensor.class, "nearPixelColorSensor");
    }

    @Override
    public void startRobot() {

    }

    @Override
    public void updateLoop() {
        sleep(10);
        pixelHere = farPixelColorSensor.red() > 1000 || farPixelColorSensor.green() > 1000 || farPixelColorSensor.green() > 1000;
        telemetry.addData("pixelHere: ", pixelHere);
        telemetry.addData("Color far sensor rgb", String.format("%d, %d, %d", farPixelColorSensor.red(), farPixelColorSensor.green(), farPixelColorSensor.blue()));
        telemetry.addData("Color near sensor rgb", String.format("%d, %d, %d", nearPixelColorSensor.red(), nearPixelColorSensor.green(), nearPixelColorSensor.blue()));
        telemetry.update();
    }
}