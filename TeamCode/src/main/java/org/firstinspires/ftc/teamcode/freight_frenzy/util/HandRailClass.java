package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HandRailClass {
    private DcMotorEx rail = null;
    private DcMotorEx hand = null;
    private CRServo grabber_right = null;
    private CRServo grabber_left = null;
    private DigitalChannel grabber_switch = null;
    private DigitalChannel hand_limit_front = null;
    private DigitalChannel hand_limit_back = null;
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
        hand_limit_front = hw.get(DigitalChannel.class, "hand_limit_front");
        hand_limit_back = hw.get(DigitalChannel.class, "hand_limit_back");

    }

    public void grab() {

    }

    public void rail_move(double pow) {
       rail.setPower(pow);
    }
}



