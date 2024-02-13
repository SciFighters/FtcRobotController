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
    private final ArrayList<Waypoint> path;
    public Location startLocation;
    private int currentLocationIndex = 0;

    /**
     * Constructs an AutoPath object with the specified drive system, start location, and waypoints.
     *
     * @param drive_        The DriveClass instance to control robot movement.
     * @param startLocation The starting location of the robot.
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
     * @throws RuntimeException if steps are negative.
     */
    public void step(int steps) {
        if (steps < 0) {
            throw new RuntimeException("Cannot go negative amount of steps");
        } else {
            for (int i = 0; i < steps; i++) {
                Waypoint current = path.get(currentLocationIndex);
                current.run();
                prevLocation = prevLocation.add(current.locationDelta);
                skip();
            }
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
     * Flips the Y axis of all waypoints in the path.
     */
    public void flipY() {
        for (int i = 0; i < path.size(); i++) {
            Waypoint newW = path.get(i);
            newW.locationDelta.flipY();
            path.set(i, newW);
        }
    }

    /**
     * Flips the X axis of all waypoints in the path.
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
        public Type type = Type.Step;

        /**
         * Constructs a Waypoint object with the specified location and movement parameters.
         *
         * @param locationDelta The target location of the waypoint.
         * @param power         The power level for movement.
         * @param tolerance     The tolerance radius around the target location.
         * @param timeout       The timeout duration for reaching the target location.
         * @param noSlowdown    Whether to disable slowdown mode.
         * @param type          The type of waypoint.
         */
        public Waypoint(Location locationDelta, double power, double tolerance, double timeout, boolean noSlowdown, Type type) {
            this.locationDelta = locationDelta;
            this.type = type;
            this.settings = new DriveClass.GotoSettings.Builder().setPower(power).setTolerance(tolerance).setTimeout(timeout).setSlowdownMode(noSlowdown).build();
        }

        /**
         * Constructs a Waypoint object with the specified location and movement settings.
         *
         * @param locationDelta The target location of the waypoint.
         * @param settings      The movement settings for reaching the target location.
         * @param type          The type of waypoint.
         */
        public Waypoint(Location locationDelta, DriveClass.GotoSettings settings, Type type) {
            this.locationDelta = locationDelta;
            this.settings = settings;
            this.type = type;
        }

        /**
         * Executes movement to reach the waypoint's location using the specified settings.
         */
        public void run() {
            Location location = locationDelta;
            if (locationDelta.x == 0 && locationDelta.y == 0) {
                drive.turnTo(locationDelta.angle, settings.power / 2);
                return;
            }
            if (type == Type.Step) {
                location = prevLocation.add(new Location(locationDelta.x, locationDelta.y, locationDelta.angle));
            }
            drive.goToLocation(new Location(location.x, location.y), location.angle, settings);
        }

        /**
         * Enumerates the type of waypoint.
         */
        public enum Type {
            Step, Location
        }
    }

    /**
     * Builder class for constructing AutoPath objects.
     */
    public static class Builder {
        private final ArrayList<Waypoint> waypoints = new ArrayList<>();

        /**
         * Adds a waypoint to the path with the specified movement parameters.
         *
         * @param location   The target location of the waypoint.
         * @param power      The power level for movement.
         * @param tolerance  The tolerance radius around the target location.
         * @param timeout    The timeout duration for reaching the target location.
         * @param noSlowdown Whether to disable slowdown mode.
         * @return The Builder instance.
         */
        public Builder addStep(Location location, double power, double tolerance, double timeout, boolean noSlowdown) {
            return addStep(location, new DriveClass.GotoSettings.Builder().setPower(power).setTolerance(tolerance).setTimeout(timeout).setSlowdownMode(noSlowdown).build());
        }

        /**
         * Adds a waypoint to the path with the specified movement settings.
         *
         * @param location The target location of the waypoint.
         * @param settings The movement settings for reaching the target location.
         * @return The Builder instance.
         */
        public Builder addStep(Location location, DriveClass.GotoSettings settings) {
            return add(location, settings, Waypoint.Type.Step);
        }

        /**
         * Adds a location to the path with the specified location settings.
         *
         * @param location The target location of the waypoint.
         * @param settings The movement settings for reaching the target location.
         * @return The Builder instance.
         */
        public Builder addLocation(Location location, DriveClass.GotoSettings settings) {
            return add(location, settings, Waypoint.Type.Location);
        }

        /**
         * Cuts a section of the path defined by indices from 'from' to 'to'.
         *
         * @param from The starting index of the section to cut.
         * @param to   The ending index of the section to cut.
         * @param path The AutoPath object from which to cut the section.
         * @return A new Builder instance containing the cut section.
         */
        public Builder cut(int from, int to, AutoPath path) {
            Builder builder = new Builder();
            for (int i = from; i <= to; i++) {
                Waypoint waypoint = path.path.get(i);
                builder.add(waypoint.locationDelta, waypoint.settings, waypoint.type);
            }
            return builder;
        }

        /**
         * Combines another Builder instance with this one.
         *
         * @param other The other Builder instance to combine.
         * @return This Builder instance after combining.
         */
        public Builder combine(Builder other) {
            for (int i = 0; i < other.waypoints.size(); i++) {
                Waypoint waypoint = other.waypoints.get(i);
                this.add(waypoint.locationDelta, waypoint.settings, waypoint.type);
            }
            return this;
        }

        private Builder add(Location location, DriveClass.GotoSettings settings, Waypoint.Type type) {
            waypoints.add(new Waypoint(location, settings, type));
            return this;
        }

        /**
         * Constructs an AutoPath object with the specified drive system and start location.
         *
         * @param drive         The DriveClass instance to control robot movement.
         * @param startLocation The starting location of the robot.
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
