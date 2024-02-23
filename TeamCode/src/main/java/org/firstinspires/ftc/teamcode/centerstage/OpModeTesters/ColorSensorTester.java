package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;


@TeleOp(group = "TESTER")
public class ColorSensorTester extends Robot {
    ColorSensor farPixelColorSensor;
    boolean pixelHere;

    @Override
    public void initRobot() {
        farPixelColorSensor = hardwareMap.get(ColorSensor.class, "armColorSensor");
    }

    @Override
    public void startRobot() {

    }

    @Override
    public void updateLoop() {
        sleep(10);
        if (farPixelColorSensor.red() > 1000 || farPixelColorSensor.green() > 1000 || farPixelColorSensor.green() > 1000) {
            pixelHere = true;
        } else {
            pixelHere = false;
        }
        telemetry.addData("pixelHere: ", pixelHere);
        telemetry.addData("Color sensor rgb", String.format("%d, %d, %d", farPixelColorSensor.red(), farPixelColorSensor.green(), farPixelColorSensor.blue()));
        telemetry.update();
    }
}