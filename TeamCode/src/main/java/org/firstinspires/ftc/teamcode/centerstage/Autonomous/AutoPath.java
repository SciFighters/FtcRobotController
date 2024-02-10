package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Represents a sequence of waypoints to navigate through during autonomous mode.
 * This class provides methods to execute the path step by step or all at once.
 */
public class AutoPath {
    private static DriveClass drive;
    private static Location prevLocation;
    public Location startLocation;
    private ArrayList<Waypoint> path;
    private int currentLocationIndex = 0;

    /**
     * Constructs an AutoPath object with the specified drive system, start locationDelta, and waypoints.
     *
     * @param drive_        The DriveClass instance to control robot movement.
     * @param startLocation The starting locationDelta of the robot.
     * @param waypoints     The list of waypoints defining the path.
     */
    private AutoPath(DriveClass drive_, Location startLocation, Waypoint... waypoints) {
        this.path = new ArrayList<>();
        path.addAll(Arrays.asList(waypoints));
        drive = drive_;
        this.startLocation = startLocation;
        prevLocation = startLocation;
    }

    /**
     * Executes a specified number of steps in the path.
     *
     * @param steps The number of steps to execute.
     */
    public void step(int steps) {
        if (steps == 0) return;
        else if (steps < 0) {
            throw new RuntimeException("Cannot go negative amount of steps");
        }
        for (int i = 0; i < steps; i++) {
            Waypoint current = path.get(currentLocationIndex);
            current.run();
            prevLocation = prevLocation.add(current.locationDelta);
            skip();
        }
    }

    /**
     * Executes one step in the path.
     */
    public void step() {
        step(1);
    }

    /**
     * Skips a specified number of waypoints in the path.
     *
     * @param amount The number of waypoints to skip.
     */
    public void skip(int amount) {
        currentLocationIndex += amount;
    }

    /**
     * Skips one waypoint in the path.
     */
    public void skip() {
        skip(1);
    }

    /**
     * Executes the remaining steps in the path.
     */
    public void run() {
        step(path.size() - currentLocationIndex);
    }

    /**
     * Flips the Y axis
     */
    public void flipY() {
        for (int i = 0; i < path.size(); i++) {
            Waypoint newW = path.get(i);
            newW.locationDelta.flipY();
            path.set(i, newW);
        }
    }

    /**
     * Flips the X axis
     */
    public void flipX() {
        for (Waypoint waypoint : path.toArray(new Waypoint[0])) {
            waypoint.locationDelta.flipX();
        }
    }

    /**
     * Represents a single waypoint in the path.
     */
    public static class Waypoint {
        public Location locationDelta;
        public DriveClass.GotoSettings settings;

        /**
         * Constructs a Waypoint object with the specified locationDelta and movement settings.
         *
         * @param locationDelta The target locationDelta of the waypoint.
         * @param power         The power level for movement.
         * @param tolerance     The tolerance radius around the target locationDelta.
         * @param timeout       The timeout duration for reaching the target locationDelta.
         * @param noSlowdown    Whether to disable slowdown mode.
         */
        public Waypoint(Location locationDelta, double power, double tolerance, double timeout, boolean noSlowdown) {
            this.locationDelta = locationDelta;
            this.settings = new DriveClass.GotoSettings.Builder().setPower(power).setTolerance(tolerance).setTimeout(timeout).setSlowdownMode(noSlowdown).build();
        }

        /**
         * Constructs a Waypoint object with the specified locationDelta and movement settings.
         *
         * @param locationDelta The target locationDelta of the waypoint.
         * @param settings      The movement settings for reaching the target locationDelta.
         */
        public Waypoint(Location locationDelta, DriveClass.GotoSettings settings) {
            this.locationDelta = locationDelta;
            this.settings = settings;
        }

        /**
         * Executes movement to reach the waypoint's locationDelta using the specified settings.
         */
        public void run() {
            if (locationDelta.x == 0 && locationDelta.y == 0) {
                drive.turnTo(locationDelta.angle, settings.power);
                return;
            }
            drive.goToLocation(prevLocation.add(new Location(locationDelta.x, locationDelta.y)), locationDelta.angle, settings);
        }
    }

    /**
     * Builder class for constructing AutoPath objects.
     */
    public static class Builder {
        private static ArrayList<Waypoint> waypoints = new ArrayList<>();

        /**
         * Adds a waypoint to the path with the specified movement parameters.
         *
         * @param location   The target locationDelta of the waypoint.
         * @param power      The power level for movement.
         * @param tolerance  The tolerance radius around the target locationDelta.
         * @param timeout    The timeout duration for reaching the target locationDelta.
         * @param noSlowdown Whether to disable slowdown mode.
         * @return The Builder instance.
         */
        public Builder addStep(Location location, double power, double tolerance, double timeout, boolean noSlowdown) {
            return addStep(location, new DriveClass.GotoSettings.Builder().setPower(power).setTolerance(tolerance).setTimeout(timeout).setSlowdownMode(noSlowdown).build());
        }

        /**
         * Adds a waypoint to the path with the specified movement settings.
         *
         * @param location The target locationDelta of the waypoint.
         * @param settings The movement settings for reaching the target locationDelta.
         * @return The Builder instance.
         */
        public Builder addStep(Location location, DriveClass.GotoSettings settings) {
            waypoints.add(new Waypoint(location, settings));
            return this; // Return the Builder instance
        }

        /**
         * Constructs an AutoPath object with the specified drive system and start locationDelta.
         *
         * @param drive         The DriveClass instance to control robot movement.
         * @param startLocation The starting locationDelta of the robot.
         * @return The constructed AutoPath object.
         */
        public AutoPath build(DriveClass drive, Location startLocation) {
            try {
                return new AutoPath(drive, startLocation, waypoints.toArray(new Waypoint[0]));
            } catch (Exception e) {
                return null;
            }
        }
    }
}
