package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.freight_frenzy.study.DuckLine;

// Handrail class
public class HandRailClass {
    //motors, servos and touch switch
    private DcMotorEx rail = null;
    private DcMotorEx hand = null;
    private CRServo grabber_right = null;
    private CRServo grabber_left = null;
    private DigitalChannel grabber_switch = null;
    private DigitalChannel hand_limit_B= null;
    private DigitalChannel hand_limit_F= null;
    private DigitalChannel rail_limit_B = null;
    private DigitalChannel rail_limit_F = null;

    private DcMotorEx carousel = null;


    public enum State {
        Drive,
        Goto
    }

    public State state = State.Drive;
    private int RAIL_MAX = 1000;

    private LinearOpMode opMode;

    public HandRailClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        rail = hw.get(DcMotorEx.class, "rail");// Getting from hardware map
        hand = hw.get(DcMotorEx.class, "hand");

        rail.setDirection(DcMotorEx.Direction.REVERSE);// Setting directions
        hand.setDirection(DcMotorEx.Direction.FORWARD);

        rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);// Setting encoders
        hand.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        grabber_right = hw.get(CRServo.class, "grabber_right");
        grabber_left = hw.get(CRServo.class, "grabber_left");

        grabber_left.setDirection(CRServo.Direction.FORWARD);
        grabber_right.setDirection(CRServo.Direction.REVERSE);

        grabber_switch = hw.get(DigitalChannel.class, "grabber_switch");
//        hand_limit = hw.get(DigitalChannel.class, "hand_limit_front");
        rail_limit_B = hw.get(DigitalChannel.class, "rail_limit_back");
        rail_limit_F = hw.get(DigitalChannel.class, "rail_limit_front");

        hand_limit_B = hw.get(DigitalChannel.class, "hand_limit_back");
        hand_limit_F = hw.get(DigitalChannel.class, "hand_limit_front");

        carousel = hw.get(DcMotorEx.class, "carousel");
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
        if (grabber_switch.getState()) {
            setServosPower(1);
        }
        else {
            setServosPower(0);
        }
    }

    public void grabberStop() {
        setServosPower(0.0);
    }

    public void grabberRelease() {
        setServosPower(-0.6);
    }

    public void rail_drive(double power) {
        if(Math.abs(power) > 0.1) {
            setState(State.Drive);
            if (power > 0 && rail_limit_F.getState())
                rail.setPower(power);
            else if (power < 0 && rail_limit_B.getState())
                rail.setPower(power);
            else
                rail.setPower(0);
        }
    }

    public void hand_drive(double power) {
        if(Math.abs(power) > 0.1) {
            setState(State.Drive);
            if(power > 0 && hand_limit_F.getState())
                hand.setPower(power);
            else if (power < 0 && hand_limit_B.getState())
                hand.setPower(power);
            else
                hand.setPower(0);
        }
    }


    public void resetPos(int pos) { // reset rail's position to it's optimized or initial position1
        rail.setPower(0.0);
        rail.setTargetPosition(pos);
    }

    public boolean grabber_ts() { // grabber touch switch update (limit of the grabber), and returns grabber state (pressed / unpressed)
        /*if(grabber_switch.getState()) {
            // updates
            stop(); // if touch switch is pressed then stop

        }*/
        return !grabber_switch.getState();
    }


    public void searchHomeRail(){
        int lastPos = rail.getCurrentPosition();
        int tempPos = lastPos;
        int counter = 0;

        //rail limit
        rail.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lastPos = rail.getCurrentPosition();

        while (rail_limit_F.getState() && rail_limit_B.getState()){
            rail.setPower(-0.5);
//            opMode.sleep(250);
            int currentPos = rail.getCurrentPosition();
//            if (currentPos >= 1000){
//                break;
//            }
//            if (lastPos >= currentPos){
//                break;
//            }
            opMode.telemetry.addData("[Homming] delta Ticks: ", Math.abs(tempPos - lastPos));
            opMode.telemetry.update();
            lastPos = currentPos;
        }
        rail.setPower(0);
        rail.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void searchHomeHand(){
        //rail got middle
        opMode.telemetry.addData("[Homing] Start: ", hand.getCurrentPosition());
        opMode.telemetry.update();
        opMode.sleep(1000);

        gotoRail(75, 0.7);
        while (rail.isBusy());
        rail.setPower(0);

        hand.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setState(State.Drive);

        opMode.telemetry.addData("[Homing] Begin search: ", hand.getCurrentPosition());
        opMode.telemetry.update();

        opMode.sleep(5000);
        // Hand go to limiter
        while(hand_limit_F.getState() && hand_limit_B.getState()) {
            hand.setPower(0.6);
            opMode.telemetry.addData("[Homing] Ticks: ", hand.getCurrentPosition());
            opMode.telemetry.update();
        }
        hand.setPower(0);

        opMode.telemetry.addData("[Homing] Found: ", hand.getCurrentPosition());
        opMode.telemetry.update();

        hand.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hand.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void gotoRail(double railPercents, double power) {
        // GOTO
        int RAIL_RANGE = 1470;
        int railTicks = (int)(railPercents/100 * RAIL_RANGE + 0.5);
        if(this.rail.getCurrentPosition() != railTicks) {
            hand.setTargetPosition(hand.getCurrentPosition());
            rail.setTargetPosition(railTicks);
            rail.setPower(power);
        }
        this.setState(State.Goto);
    }

    public  void goToABC(DuckLine.ABC abc){
        if (abc == abc.A){
            gotoHand(0,20,1);
        }else if (abc == abc.A){
            gotoHand(0,40,1);
        }else if (abc == abc.C){
            gotoHand(0,60,1);
        }
    }

    public void gotoA(){
        gotoHand(0,20,1);
    }

    public void gotoB(){
        gotoHand(0,40,1);
    }

    public void gotoC(){
        gotoHand(0,60,1);
    }

    public void gotoX(){
        gotoHand(0,10,1);
    }


    public void gotoHand(int railPos, int handPos, float pow) {
        // GOTO
        handPos = (int)this.clamp((double)handPos, 100, 0);
        if(this.hand.getCurrentPosition() != handPos) {
            hand.setTargetPosition(handPos);
            hand.setPower(pow);
        }
        this.setState(State.Goto);
    }
    //updates
    public void update_handRail() {

        opMode.telemetry.addData("hand position: ", this.hand.getCurrentPosition());
        opMode.telemetry.addData("rail position: ", this.rail.getCurrentPosition());
    }

    public void setState(State state) {
        if(this.state != state) {
            this.state = state;
            if(state == State.Drive)
                this.rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.hand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                opMode.telemetry.addData("handRail","DRIVE");
            if(state == State.Goto)
                this.rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                opMode.telemetry.addData("handRail","GOTO");
        }
    }

    public void carouselRun(double power) {

        carousel.setPower(power);
    }

    public void carouselStop() {

        carousel.setPower(0);
    }



}



