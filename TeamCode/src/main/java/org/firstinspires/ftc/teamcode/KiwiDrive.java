package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class KiwiDrive{
    private final DcMotor wheel1;
    private final DcMotor wheel2;
    private final DcMotor wheel3;
    private double moveSpeedModifier;
    private double angularSpeedModifier;
    private final IMU imu;

    public KiwiDrive(DcMotor wheel1, DcMotor wheel2, DcMotor wheel3, IMU imu){
        this.wheel1 = wheel1;
        this.wheel2 = wheel2;
        this.wheel3 = wheel3;
        this.imu = imu;
        moveSpeedModifier = 0.2;
        angularSpeedModifier = 0.25;

    }
    public void drive(double y, double x, double av){
        double[] cords = rotateVector(x,y);
        double xv = cords[0];
        double yv = cords[1];
        wheel1.setPower(Range.clip(moveSpeedModifier * (-yv * 0 + xv * 1) + av * 0.18 * angularSpeedModifier,-1,1));
        wheel2.setPower(Range.clip(moveSpeedModifier * (-yv * -Math.sqrt(3)/2 + xv * -1/2) + av * 0.18 * angularSpeedModifier,-1,1));
        wheel3.setPower(Range.clip(moveSpeedModifier * (-yv * Math.sqrt(3)/2 + xv * -1/2) + av * 0.18 * angularSpeedModifier,-1,1));
    }
    public void setMoveSpeedModifier(double moveSpeedModifier){
        this.moveSpeedModifier = moveSpeedModifier;
    }
    public void dpadMovement(boolean down, boolean up, boolean left, boolean right, double av){
        if(down) {
            if (left) {drive(0.3, -0.3, av);}
            else if (right) {drive(0.3, 0.3, av);}
            else {drive(0.3,0,av);}
        }
        else if(up) {
            if (left) {drive(-0.3, -0.3, av);}
            else if (right) {drive(-0.3, 0.3, av);}
            else {drive(-0.3,0,av);}
        }
        else if(left) {drive(0,-0.3,av);}
        else if(right) {drive(0,0.3,av);}
    }
    private double[] rotateVector(double a, double b){
        double angle;
        double[] cords = {0,0};
        if (b == 0){
            angle = 90 + 90 * a / Math.abs(a);
        }
        else{angle = b/Math.abs(b)* Math.acos(a/Math.sqrt(a*a + b*b));}
        cords[0] = Math.sqrt(a*a + b*b) * Math.cos(angle + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        cords[1] = Math.sqrt(a*a + b*b) * Math.sin(angle + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        return cords;
    }
    public void setBoost(boolean left, boolean right) {
        if(left && right){
            setMoveSpeedModifier(0.4);
        }
        else if (left || right){
            setMoveSpeedModifier(0.3);
        }
        else {
            setMoveSpeedModifier(0.2);
        }
    }
}