package org.firstinspires.ftc.teamcode.centerstage.util;

import org.firstinspires.ftc.teamcode.power_play.util.DriveClass;

public class Location {

    public double x, y, angle = 0;

    public Location(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Location(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    public Location(Location location) {
        this.x = location.x;
        this.y = location.y;
        this.angle = location.angle;
    }

    public Location(Location location, double xOffset, double yOffset, double angleOffset) {
        this.x = location.x + xOffset;
        this.y = location.y + yOffset;
        this.angle = location.angle + angleOffset;
    }

    public void flipX() {
        this.x *= -1;
    }

    public void flipY() {
        this.y *= -1;
    }

    public void flipAngle() {
        this.angle *= -1;
    }

    public Location offsetY(double offset) {
        return new Location(this.x, this.y + offset, this.angle);
    }

    public Location add(Location location) {
        addX(location.x);
        addY(location.y);
        return this;
    }

    public Location addX(double x) {
        this.x += x;
        return this;
    }

    public Location addY(double y) {
        this.y += y;
        return this;
    }

    public Location subtract(Location location) {
        subtractX(location.x);
        subtructY(location.y);
        return this;
    }

    public Location subtractX(double x) {
        this.x -= x;
        return this;
    }

    public Location subtructY(double y) {
        this.y -= y;
        return this;
    }

    public Location multiply(float num) {
        multiplyX(num);
        multiplyY(num);
        return this;
    }

    public Location multiplyX(double x) {
        this.x *= x;
        return this;
    }

    public Location multiplyY(double y) {
        this.y *= y;
        return this;
    }

    public Location offsetX(double offset) {
        return new Location(this.x + offset, this.y, this.angle);
    }

    public Location stayOnX(org.firstinspires.ftc.teamcode.power_play.util.DriveClass drive) {
        return new Location(drive.getPosX(), this.y);
    }

    public Location stayOnY(DriveClass drive) {
        return new Location(this.x, drive.getPosY());
    }

}
