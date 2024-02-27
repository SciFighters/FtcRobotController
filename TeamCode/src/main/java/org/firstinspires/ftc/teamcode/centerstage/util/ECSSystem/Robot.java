package org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

/**
 * The abstract base class for creating robot programs.
 * Extend this class to create a custom robot program.
 *
 * @see LinearOpMode
 */
public abstract class Robot extends LinearOpMode {
    private final Map<Component, Thread> components = new HashMap<>();
    public AutoFlow.Alliance alliance = AutoFlow.Alliance.BLUE;
    public TYPE type;
    public ElapsedTime elapsedTime;
    protected Telemetry robotTelemetry = telemetry;

    /**
     * Runs the main loop of the robot program.
     * Override this method to define the robot's behavior.
     *
     * @throws InterruptedException If the program is interrupted
     */
    @Override
    public final void runOpMode() throws InterruptedException {
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        getTelemetry();
        initRobot();
        waitForStart();
        startComponents();
        startRobot();
        startComponentThreads();
        while (opModeIsActive() && !isStopRequested()) {
            updateLoop();
            components.forEach((c, t) -> {
                if (t == null && c.enabled) c.update();
            });
        }

        stopComponentThreads();
    }

    /**
     * Initializes the robot. Override this method to perform any necessary
     * initialization steps for the robot.
     */
    public abstract void initRobot();

    /**
     * Starts the robot. Override this method to define actions that should
     * be taken when the robot starts running.
     */
    public abstract void startRobot();

    /**
     * The main update loop of the robot. Override this method to define the
     * behavior that should be executed repeatedly during the program.
     */
    public void updateLoop() {
//        requestOpModeStop();
    }

    /**
     * Stops the robot. Override this method to define actions that should be
     * taken when the robot is stopped.
     */
    public void onStop() {
    }

    /**
     * Adds a component to the robot.
     *
     * @param <T>               The type of the component
     * @param componentClass    The class of the component
     * @param componentInstance The instance of the component to add
     * @return The added component or null if an error occurred
     */
    public <T extends Component> T addComponent(Class<T> componentClass, T componentInstance) {
        if (componentInstance == null) {
            throw new IllegalArgumentException("Component instance cannot be null");
        }

        try {
            initializeComponent(componentInstance); // <-- Potential cause of NullPointerException
            return getComponent(componentClass);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }


    /**
     * Adds a component to the robot.
     *
     * @param <T>            The type of the component
     * @param componentClass The class of the component
     * @return The added component or null if an error occurred
     */
    public <T extends Component> T addComponent(Class<T> componentClass) {
        T componentInstance = createComponentInstance(componentClass);
        return addComponent(componentClass, componentInstance);
    }

    /**
     * Gets a component of the specified type.
     *
     * @param <T>            The type of the component
     * @param componentClass The class of the component
     * @return The component of the specified type or null if not found
     */
    public <T extends Component> T getComponent(Class<T> componentClass) {
        for (Component component : components.keySet()) {
            if (componentClass.isInstance(component)) {
                return componentClass.cast(component);
            }
        }
        return null;
    }

    private void startComponents() {
        components.keySet().forEach(Component::start);
    }

    private void startComponentThreads() {
        components.values().forEach(t -> {
            if (t != null) t.start();
        });
    }

    private void stopComponentThreads() {
        components.values().forEach(t -> {
            if (t != null) t.interrupt();
        });
    }

    private void initializeComponent(Component component) {
        if (component == null) {
            throw new NullPointerException("COMPONENT IS NULL");
        }
        component.attach(this, robotTelemetry);
        Thread thread = createComponentThread(component);
        components.put(component, thread);
        component.init();
    }

    private Thread createComponentThread(Component component) {
        return component.getClass().isAnnotationPresent(ThreadedComponent.class) ? new Thread(component) : null;
    }

    private <T extends Component> T createComponentInstance(Class<T> componentClass) {
        try {
            return componentClass.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException("Error creating component instance", e);
        }
    }

    /**
     * Finds and sets the robot telemetry field using reflection.
     * This method is called during initialization to set the telemetry object.
     */
    void getTelemetry() {
        if (this.getClass().isAnnotationPresent(Autonomous.class)) {
            this.type = TYPE.Auto;
        } else {
            this.type = TYPE.TeleOp;
        }
        for (Field f : getClass().getDeclaredFields()) {
            try {
                f.setAccessible(true);
                if (f.isAnnotationPresent(RobotTelemetry.class) && f.get(this) instanceof Telemetry) {
                    robotTelemetry = (Telemetry) f.get(this);
                }
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public enum TYPE {
        Auto, TeleOp
    }
}
