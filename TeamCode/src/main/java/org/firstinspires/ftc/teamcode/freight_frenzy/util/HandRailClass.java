package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

    private AnalogInput potentiometer = null;

    private DcMotorEx carousel = null;

    private int railRange = 1470;


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
//       hand_limit = hw.get(DigitalChannel.class, "hand_limit_front");
        rail_limit_B = hw.get(DigitalChannel.class, "rail_limit_back");
        rail_limit_F = hw.get(DigitalChannel.class, "rail_limit_front");

        hand_limit_B = hw.get(DigitalChannel.class, "hand_limit_back");
        hand_limit_F = hw.get(DigitalChannel.class, "hand_limit_front");

        carousel = hw.get(DcMotorEx.class, "carousel");

        potentiometer = hw.get(AnalogInput.class, "potentiometer");
    }


    public enum gotoPoints {
        pointA(0,0),
        pointB(0,0),
        pointC(0,0),
        pointX(0,0);

        public double railPower = 0.8;
        public float handPower = (float)0.8;
        public double railPercentage;
        public int handPercentage;


        gotoPoints(double railPercentage, int handPercentage) {
            this.railPercentage = railPercentage;
            this.handPercentage = handPercentage;
        }
    }


    public void gotoPoint(gotoPoints point) {
        this.gotoRail(point.railPercentage, point.railPower);
        this.gotoPercent(0, point.handPercentage, point.handPower);
    }


    public enum State {
        Drive,
        Goto
    }

    public State state = State.Drive;
    private LinearOpMode opMode;

    public HandRailClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public int getRailPos(){
        int pos = rail.getCurrentPosition();
        int railTicks = (int)(pos/100 * railRange + 0.5);
        return railTicks;
    }


    public double getPotentiometerValue(){
        return potentiometer.getVoltage();
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
        if(Math.abs(power) > 0.1 || rail.isBusy() == false) {
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
        if(Math.abs(power) > 0.1 || hand.isBusy() == false) {
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
            rail.setPower(-0.8);
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

        gotoRail(75, 0.7);
        while (rail.isBusy());
        rail.setPower(0);

        hand.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //setState(State.Drive);

        opMode.telemetry.addData("[Homing] Begin search: ", hand.getCurrentPosition());
        opMode.telemetry.update();

        // Hand go to limiter
        while(hand_limit_F.getState() && hand_limit_B.getState()) {
            hand.setPower(0.6);
            opMode.telemetry.addData("[Homing] Ticks: ", hand.getCurrentPosition());

            opMode.telemetry.addData("[Homing]", "tmp in loop");
            opMode.telemetry.addData("target:", hand.getTargetPosition());
            opMode.telemetry.addData("cur:", hand.getCurrentPosition());
            opMode.telemetry.addData("cur mode:", hand.getMode());
            opMode.telemetry.addData("cur:", hand.getCurrentPosition());
            opMode.telemetry.addData("cur mode:", hand.getMode());
            opMode.telemetry.addData("hand_front:", hand_limit_F.getState());
            opMode.telemetry.addData("hand_back:", hand_limit_B.getState());
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
        int railTicks = (int)(railPercents/100 * railRange + 0.5);
        if(this.rail.getCurrentPosition() != railTicks) {
            hand.setTargetPosition(hand.getCurrentPosition());
            rail.setTargetPosition(railTicks);
            rail.setPower(power);
        }
        this.setState(State.Goto);
    }

    public  void goToABC(DuckLine.ABC abc){
        if (abc == abc.A){
            gotoPercent(100,90,1);
        }else if (abc == abc.B){
            gotoPercent(100,50,1);
        }else if (abc == abc.C){
            gotoPercent(100,70,1);
        } else if (abc == abc.X){
            gotoPercent(100,5,1);
        }
    }

    public void gotoPercent(int railPos, int handPos, double power) {
        // GOTO
        handPos = (int)this.clamp((double)handPos, 100, 0);
        if(this.hand.getCurrentPosition() != handPos) {
            rail.setTargetPosition(railPos);
            rail.setPower(power);
            hand.setTargetPosition(handPos);
            hand.setPower(power);

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
            if (state == State.Drive) {
                this.rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.hand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                opMode.telemetry.addData("handRail", "DRIVE");
            } else if (state == State.Goto) {
                this.rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                opMode.telemetry.addData("handRail", "GOTO");
            }
        }
    }

    public void carouselRun(double power) {

        carousel.setPower(power);
    }

    public void carouselStop() {

        carousel.setPower(0);
    }

    public void switchSides(){
        setState(State.Goto);
        int pos = getRailPos();
        if (pos < 50){
            gotoRail(100, 1);
        }
        else{
            gotoRail(0, 1);
        }
    }
}



