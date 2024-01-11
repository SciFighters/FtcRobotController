package org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public abstract class Robot extends LinearOpMode {
    Set<Component> components = new HashSet<>();

    @Override
    public final void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        startRobot();
        while (opModeIsActive() && !isStopRequested()) {
            updateLoop();
            for (Component c : components) {
                c.loop();
            }
        }
    }

    public void updateLoop() {

    }

    public void initRobot() {

    }

    public void startRobot() {

    }

    public void stopRobot() {
    }

    public <T extends Component> void addComponent(Class<T> componentClass) {
        try {
            T componentInstance = componentClass.getDeclaredConstructor().newInstance();
            componentInstance.attach(this);
            this.components.add(componentInstance);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public <T extends Component> T getComponent(Class<T> componentClass) {
        for (Component component : components) {
            if (componentClass.isInstance(component)) {
                return componentClass.cast(component);
            }
        }
        return null; // Return null if the component is not found
    }
}
