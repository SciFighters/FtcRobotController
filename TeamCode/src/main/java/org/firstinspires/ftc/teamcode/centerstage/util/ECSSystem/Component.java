package org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Component implements Runnable {
    public Robot robot;
    public HardwareMap hw;
    public Telemetry telemetry;
    public boolean enabled;

    public abstract void init();

    public void start() {
    }

    public void loop() {
        return;
    }

    public void stop() {
        return;
    }

    public final void run() {
        while (robot.opModeIsActive() && !robot.isStopRequested()) {
            loop();
            robot.idle();
        }
    }

    public void attach(Robot robot, Telemetry telemetry) {
        this.robot = robot;
        this.hw = robot.hardwareMap;
        this.telemetry = telemetry;
    }
}
