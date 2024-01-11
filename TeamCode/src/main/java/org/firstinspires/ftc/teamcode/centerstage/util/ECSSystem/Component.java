package org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem;

public abstract class Component {
    public Robot robot;
    public boolean enabled;

    abstract void init();

    abstract void loop();

    abstract void stop();

    void attach(Robot robot) {
        this.robot = robot;
    }
}
