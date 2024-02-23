package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
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
     * @param robot         The robot instance to control robot movement.
     * @param startLocation The starting location of the robot.
     * @param waypoints     The list of waypoints defining the path.
     * @see Waypoint
     */
    private AutoPath(Robot robot, Location startLocation, Waypoint... waypoints) {
        this.path = new ArrayList<>();
        path.addAll(Arrays.asList(waypoints));
        drive = robot.getComponent(DriveClass.class);
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
        if (steps == 0) return;

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
            if (newW.type == Waypoint.Type.StaticWaypoint)
                continue;
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
         * Constructs a Waypoint object with the specified location and movement settings.
         *
         * @param locationDelta The target location of the waypoint.
         * @param settings      The movement settings for reaching the target location.
         * @param type          The type of waypoint.
         * @see Location
         * @see org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass.GotoSettings
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
                location.angle = locationDelta.angle;
            }
            drive.goToLocation(location, settings);
        }

        /**
         * Enumerates the type of waypoint.
         */
        public enum Type {
            Step, Location, StaticWaypoint
        }
    }

    /**
     * Builder class for constructing AutoPath objects.
     */
    public static class Builder {
        private final ArrayList<Waypoint> waypoints = new ArrayList<>();

        /**
         * Adds a waypoint to the path with the specified movement settings.
         *
         * @param location The target location of the waypoint.
         * @param settings The movement settings for reaching the target location.
         * @return The {@link Builder} instance.
         * @see Location
         */
        public Builder addStep(Location location, DriveClass.GotoSettings settings) {
            return add(location, settings, Waypoint.Type.Step);
        }

        /**
         * Adds a location to the path with the specified location settings.
         *
         * @param location The target location of the waypoint.
         * @param settings The movement settings for reaching the target location.
         * @return The {@link Builder} instance.
         * @see Location
         */
        public Builder addLocation(Location location, DriveClass.GotoSettings settings) {
            return add(location, settings, Waypoint.Type.Location);
        }

        /**
         * Adds a static waypoint (isn't getting flipped) to the path with the specified location settings
         *
         * @param location The target location of the waypoint
         * @param settings The movement settings for reaching the target location
         * @return The {@link Builder} instance.
         * @see Location
         */
        public Builder addStaticWaypoint(Location location, DriveClass.GotoSettings settings) {
            return add(location, settings, Waypoint.Type.StaticWaypoint);
        }

        /**
         * Cuts a section of the path defined by indices from 'from' to 'to'.
         *
         * @param from The starting index of the section to cut.
         * @param to   The ending index of the section to cut.
         * @param path The {@link AutoPath} object from which to cut the section.
         * @return A new {@link Builder} instance containing the cut section.
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
         * Combines another {@link Builder} instance with this one.
         *
         * @param other The other {@link Builder} instance to combine.
         * @return This {@link Builder} instance after combining.
         */
        public Builder combine(Builder other) {
            for (int i = 0; i < other.waypoints.size(); i++) {
                Waypoint waypoint = other.waypoints.get(i);
                this.add(waypoint.locationDelta, waypoint.settings, waypoint.type);
            }
            return this;
        }

        /**
         * Adds a new waypoint with the specified type, location and settings
         *
         * @param location The target location
         * @param settings Movement settings for reaching the target location
         * @param type     The waypoint type : Step (A movement), Location (a specific place) or a Static Waypoint (a specific place that isn't flipped together with the rest of the path.
         * @return The {@link Builder} instance.
         * @see #addLocation(Location, DriveClass.GotoSettings)
         * @see #addStep(Location, DriveClass.GotoSettings)
         * @see #addStaticWaypoint(Location, DriveClass.GotoSettings)
         */
        private Builder add(Location location, DriveClass.GotoSettings settings, Waypoint.Type type) {
            waypoints.add(new Waypoint(location, settings, type));
            return this;
        }

        /**
         * Constructs an AutoPath object with the specified drive system and start location.
         *
         * @param robot         The {@link Robot} instance to control robot movement.
         * @param startLocation The starting location of the robot.
         * @return The constructed AutoPath object.
         */
        public AutoPath build(Robot robot, Location startLocation) {
            try {
                return new AutoPath(robot, startLocation, waypoints.toArray(new Waypoint[0]));
            } catch (Exception e) {
                return null;
            }
        }
    }
}
