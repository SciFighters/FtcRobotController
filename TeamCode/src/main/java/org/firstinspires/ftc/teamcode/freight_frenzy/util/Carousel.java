package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {

    private LinearOpMode opMode;
    private DcMotor carousel = null;

    public void init(HardwareMap hw) {
        carousel = hw.get(DcMotorEx.class, "carousel");
    }

    public Carousel(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void start(float p) {
        carousel.setPower(p);
    }

    public void stop() {
        carousel.setPower(0);
    }
}
