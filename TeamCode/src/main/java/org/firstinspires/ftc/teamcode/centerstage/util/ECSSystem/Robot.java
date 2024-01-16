package org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

/**
 * The base class for robot implementations in the ECSSystem.
 * Provides a framework for managing robot components as separate threads.
 */
public abstract class Robot extends LinearOpMode {

    private final Map<Component, Thread> components = new HashMap<>();
    protected Telemetry robotTelemetry = null;

    /**
     * The main run loop for the robot. Initializes, starts, and updates the robot components.
     *
     * @throws InterruptedException if the run loop is interrupted
     */
    @Override
    public final void runOpMode() throws InterruptedException {
        getTelemetry();
        initRobot();
        waitForStart();
        startRobot();
        startComponentThreads();
        while (opModeIsActive() && !isStopRequested()) {
            updateLoop();
        }
        stopComponentThreads();
    }

    /**
     * Placeholder for the user-defined update loop.
     */
    public abstract void updateLoop();

    /**
     * Placeholder for user-defined robot initialization logic.
     */
    public abstract void initRobot();

    /**
     * Placeholder for user-defined robot start logic.
     */
    public abstract void startRobot();

    /**
     * Placeholder for user-defined robot stop logic.
     */
    public void stopRobot() {
        // Add custom stop logic here
    }

    /**
     * Adds a component to the robot and creates a corresponding thread for it.
     *
     * @param <T>               the type of the component
     * @param componentClass    the class of the component
     * @param componentInstance an instance of the component
     * @return the added component or null if an error occurs
     * @see Component
     */
    public <T extends Component> T addComponent(Class<T> componentClass, T componentInstance) {
        try {
            initializeComponent(componentInstance);
            return componentInstance;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Adds a component to the robot using reflection to create an instance of the component class.
     *
     * @param <T>            the type of the component
     * @param componentClass the class of the component
     * @return the added component or null if an error occurs
     * @see #addComponent(Class, Component)
     */
    public <T extends Component> T addComponent(Class<T> componentClass) {
        T componentInstance = createComponentInstance(componentClass);
        return addComponent(componentClass, componentInstance);
    }

    /**
     * Retrieves a component of the specified class from the robot's components map.
     *
     * @param <T>            the type of the component
     * @param componentClass the class of the component
     * @return the component of the specified class or null if not found
     */
    public <T extends Component> T getComponent(Class<T> componentClass) {
        for (Component component : components.keySet()) {
            if (componentClass.isInstance(component)) {
                return componentClass.cast(component);
            }
        }
        return null;
    }

    private void startComponentThreads() {
        for (Thread thread : components.values()) {
            if (thread != null)
                thread.start();
        }
    }

    private void stopComponentThreads() {
        for (Thread thread : components.values()) {
            if (thread != null)
                thread.interrupt();
        }
    }

    private void initializeComponent(Component component) {
        component.attach(this, robotTelemetry);
        component.init();
        Thread thread = null;
        if (component.getClass().isAnnotationPresent(ThreadedComponent.class)) {
            thread = new Thread(component);
        }
        components.put(component, thread);
    }

    private <T extends Component> T createComponentInstance(Class<T> componentClass) {
        try {
            return componentClass.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException("Error creating component instance", e);
        }
    }

    void getTelemetry() {
        for (Field f : this.getClass().getDeclaredFields()) {
            try {
                f.setAccessible(true);
                if (f.isAnnotationPresent(RobotTelemetry.class) && f.get(this) instanceof Telemetry) {
                    robotTelemetry = (Telemetry) f.get(this);
                }
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
        if (robotTelemetry == null) {
            robotTelemetry = telemetry;
        }
    }
}
