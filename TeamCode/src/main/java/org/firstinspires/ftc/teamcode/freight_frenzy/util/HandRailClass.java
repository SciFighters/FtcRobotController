package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
// Handrail class
public class HandRailClass {
    //motors, servos and touch switch
    private DcMotorEx rail = null;
    private DcMotorEx hand = null;
    private CRServo grabber_right = null;
    private CRServo grabber_left = null;
    private DigitalChannel grabber_switch = null; //???
    private DigitalChannel hand_limit = null;
    private DigitalChannel rail_limit = null;




    private LinearOpMode opMode;

    public HandRailClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        rail = hw.get(DcMotorEx.class, "rail");
        hand = hw.get(DcMotorEx.class, "hand");

        grabber_right = hw.get(CRServo.class, "grabber_right");
        grabber_left = hw.get(CRServo.class, "grabber_left");

        grabber_switch = hw.get(DigitalChannel.class, "grabber_switch");
        hand_limit = hw.get(DigitalChannel.class, "hand_limit_front");
        rail_limit = hw.get(DigitalChannel.class, "hand_limit_back");
    }

    public void setServosPower(double power) {
        grabber_left.setPower(power);
        grabber_right.setPower(power);
    }

    public double clamp(double value, double maximum, double minimum) {
        if(maximum < value) value = maximum;
        if(minimum > value) value = minimum;
        return value;
    }

    public void grab() {
        setServosPower(0.95);
    }

    public void stop() {
        setServosPower(0.0);
    }

    public void release() {
        setServosPower(-0.2);
    }

    public void rail_move(double pow) {
       rail.setPower(1.0);
       rail.setTargetPosition(500);
    }


    public void resetPos(int pos) {
        rail.setPower(0.0);

    }

    public void touch_switch_update() {
        if(rail_limit.getState()) {
            //if touch switch is pressed
            rail.setPower(0.0);
            stop();
        }
    }

}



