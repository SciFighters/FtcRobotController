package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoPath {
    private ArrayList<Waypoint> path;
    public DriveClass drive;
    private int currentLocationIndex = 0;

    private AutoPath(DriveClass drive, Waypoint... waypoints) {
        this.path = new ArrayList<>();
        path.addAll(Arrays.asList(waypoints));
    }

    public void step(int steps) {
        for (int i = 0; i < steps; i++) {
            path.get(currentLocationIndex).run();
            skip();
        }
    }

    public void step() {
        step(1);
    }

    public void skip(int amount) {
        currentLocationIndex += amount;
    }

    public void skip() {
        skip(1);
    }

    public void run() {
        step(path.size() - currentLocationIndex);
    }

    public class Waypoint {
        public Location location;
        public double power, targetHeading, tolerance, timeout;
        public boolean noSlowdown;

        public Waypoint(Location location, double power, double targetHeading, double tolerance, double timeout, boolean noSlowdown) {
            this.location = location;
            this.power = power;
            this.targetHeading = targetHeading;
            this.timeout = timeout;
            this.noSlowdown = noSlowdown;
        }

        public void run() {
            drive.goToLocation(location, power, targetHeading, tolerance, timeout, noSlowdown);
        }
    }

    public class Builder {
        private ArrayList<Waypoint> waypoints = new ArrayList<>();

        public Builder addStep(Location location, double power, double targetHeading, double tolerance, double timeout, boolean noSlowdown) {
            return addInternalStep(location, power, targetHeading, tolerance, timeout, noSlowdown);
        }

        private Builder addInternalStep(Location location, double power, double targetHeading, double tolerance, double timeout, boolean noSlowdown) {
            waypoints.add(new Waypoint(location, power, targetHeading, tolerance, timeout, noSlowdown));
            return this;
        }

        public AutoPath build(DriveClass drive) {
            try {
                return new AutoPath(drive, waypoints.toArray(new Waypoint[0]));
            } catch (Exception e) {
                return null;
            }
        }
    }
}
