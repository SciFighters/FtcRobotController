package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Handrail class
public class HandRailClass {
    //motors, servos and touch switch
    private DcMotorEx rail = null;
    private DcMotorEx hand = null;
    private CRServo grabber_right = null;
    private CRServo grabber_left = null;
    private DigitalChannel grabber_switch = null;
    private DigitalChannel hand_limit_L= null;
    private DigitalChannel hand_limit_R= null;
    private DigitalChannel rail_limit_L = null;
    private DigitalChannel rail_limit_R = null;

    private int RAIL_MAX = 1000;

    private LinearOpMode opMode;

    public HandRailClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        rail = hw.get(DcMotorEx.class, "rail");
        hand = hw.get(DcMotorEx.class, "hand");

        rail.setDirection(DcMotorEx.Direction.FORWARD);
        hand.setDirection(DcMotorEx.Direction.FORWARD);

        rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hand.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        grabber_right = hw.get(CRServo.class, "grabber_right");
        grabber_left = hw.get(CRServo.class, "grabber_left");

        grabber_left.setDirection(CRServo.Direction.REVERSE);

        grabber_switch = hw.get(DigitalChannel.class, "grabber_switch");
//        hand_limit = hw.get(DigitalChannel.class, "hand_limit_front");
        rail_limit_L = hw.get(DigitalChannel.class, "rail_limit_left");
        rail_limit_R = hw.get(DigitalChannel.class, "rail_limit_right");

        searchHome();
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

    public void grabberGrab() {
        setServosPower(1);
    }

    public void grabberStop() {
        setServosPower(0.0);
    }

    public void grabberRelease() {
        setServosPower(-0.6);
    }

    public void rail_drive(double pos) {
    }


    public void resetPos(int pos) { // reset rail's position to it's optimized or initial position1
        rail.setPower(0.0);
        rail.setTargetPosition(pos);
    }

    public void railLimit_ts_L() { //rail limit (limit of the rail) touch switch
        if(rail_limit_L.getState()) {
            //if touch switch is pressed...
            rail.setPower(0.0);
            grabberStop();
        }
    }

    public void railLimit_ts_R() { //rail limit (limit of the rail) touch switch
        if(rail_limit_R.getState()) {
            //if touch switch is pressed...
            rail.setPower(0.0);
            grabberStop();
        }
    }

    public void handLimit_ts_L(){
        if (hand_limit_L.getState()){
            hand.setPower(0);
        }
    }

    public void handLimit_ts_R(){
        if (hand_limit_R.getState()){
            hand.setPower(0);
        }
    }

    public boolean grabber_ts() { // grabber touch switch update (limit of the grabber), and returns grabber state (pressed / unpressed)
        /*if(grabber_switch.getState()) {
            // updates
            stop(); // if touch switch is pressed then stop

        }*/
        return grabber_switch.getState();
    }
    public void searchHome(){
        //hand limit
        hand.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hand.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        int lastPos = hand.getCurrentPosition();
        while (hand_limit_R.getState() && hand_limit_L.getState()){
            hand.setPower(-0.5);
            opMode.sleep(250);
            int currentPos = hand.getCurrentPosition();
            if (currentPos >= 1000){
                break;
            }
            if (lastPos <= currentPos){
                break;
            }
            lastPos = currentPos;
        }
        hand.setPower(0);
        hand.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hand.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //rail limit
        rail.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lastPos = rail.getCurrentPosition();
        while (rail_limit_R.getState() && rail_limit_L.getState()){
            rail.setPower(-0.5);
            opMode.sleep(250);
            int currentPos = rail.getCurrentPosition();
            if (currentPos >= 1000){
                break;
            }
            if (lastPos <= currentPos){
                break;
            }
            lastPos = currentPos;
        }
        rail.setPower(0);
        rail.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

}



