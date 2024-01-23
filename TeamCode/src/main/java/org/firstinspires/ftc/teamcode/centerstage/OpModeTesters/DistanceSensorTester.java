package org.firstinspires.ftc.teamcode.centerstage.OpModeTesters;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

@TeleOp
public class DistanceSensorTester extends Robot {
    DistanceSensor sensor1, sensor2;

    @Override
    public void updateLoop() {
        telemetry.addData("Sensor1", sensor1.getDistance(DistanceUnit.CM));
        telemetry.addData("Sensor2", sensor2.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    @Override
    public void initRobot() {
        sensor1 = hardwareMap.get(DistanceSensor.class, "armDistanceSensor");
        sensor2 = hardwareMap.get(DistanceSensor.class, "rearLeftDistanceSensor");
    }

    @Override
    public void startRobot() {

    }
}
