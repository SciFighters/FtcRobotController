package org.firstinspires.ftc.teamcode.freight_frenzy.util;

public class Location {

	public double x;
	public double y;
	public double angle;


	public Location(double x, double y) {
		this.x = x;
		this.y = y;
	}
	public Location(double x, double y, double angle) {
		this.x = x;
		this.y = y;
		this.angle = angle;
	}

	public void flipX() {
		this.x *= -1;
	}
	public void flipY() { this.y *= -1; }
	public void flipAngle() {this.angle *= -1;}

	public Location offsetY(double offset) {
		return new Location(this.x, this.y + offset);
	}
	public Location offsetX(double offset) { return new Location(this.x + offset, this.y ); }
}
