package org.firstinspires.ftc.teamcode.freight_frenzy.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private int potentiometer_offset;

    private DcMotorEx carousel = null;

    private int railRange = 1470;
    private int handRange = 5885;

    public void init(HardwareMap hw) {
        rail = hw.get(DcMotorEx.class, "rail");// Getting from hardware map
        hand = hw.get(DcMotorEx.class, "hand");

        rail.setDirection(DcMotorEx.Direction.REVERSE);// Setting directions
        hand.setDirection(DcMotorEx.Direction.FORWARD);

        rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);// Setting encoders
        hand.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        hand.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hand.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        potentiometer_offset = getScaledPotentiometerValue();

        hand.setTargetPositionTolerance(20);
        rail.setTargetPositionTolerance(20);
        // this.searchHomeRail();
    }

    private LinearOpMode opMode;

    /**** @param linearOpMode opMode */
    public HandRailClass(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    public enum State {
        Drive(DcMotor.RunMode.RUN_USING_ENCODER),
        Goto(DcMotor.RunMode.RUN_TO_POSITION);

        DcMotor.RunMode runMode;

        State(DcMotor.RunMode runMode) {
            this.runMode = runMode;
        }
    }

    public State handState = State.Drive;
    public State railState = State.Drive;

    public void setHandState(State state){
        if(this.handState != state) {
            this.handState = state;
            if (state == State.Drive) {
                this.hand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                opMode.telemetry.addData("handRail", "DRIVE");
            } else if (state == State.Goto) {
                this.hand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                opMode.telemetry.addData("handRail", "GOTO");
            }
        }
    }

    public void setRailState(State state) {
        if(this.railState != state) {
            this.railState = state;
            if (state == State.Drive) {
                this.rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                opMode.telemetry.addData("handRail", "DRIVE");
            } else if (state == State.Goto) {
                this.rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                opMode.telemetry.addData("handRail", "GOTO");
            }
        }
    }

    public void setState (State state){
        setHandState(state);
        setRailState(state);
    }

    public void gotoRail(double railPercents, double power) {
        // GOTO rail
        int ticks = convRail_percent2ticks(railPercents);
        if(this.rail.getCurrentPosition() != ticks) {
            rail.setTargetPosition(ticks);
            rail.setPower(power);
            this.setRailState(State.Goto);
        }

    }

    public void gotoHand(double handPercents, double power) {
        //GOTO hand
        int tiks = convHand_percent2ticks(handPercents);
        if(this.hand.getCurrentPosition() != tiks) {
            hand.setTargetPosition(tiks);
            hand.setPower(power);
            this.setHandState(State.Goto);
        }
    }

    public void gotoHandRail(double railPos, double handPos, double power) {
        // GOTO handrail
        gotoHand(handPos, power);
        gotoRail(railPos, power/3);
    }

    public void gotoLevel(DuckLine.SH_Levels shLevel){
        if (shLevel == DuckLine.SH_Levels.Top) {
            gotoHandRail(65,75,1);
        } else if (shLevel == DuckLine.SH_Levels.Middle) {
            gotoHandRail(65,84,1);
        } else if (shLevel == DuckLine.SH_Levels.Bottom) {
            gotoHandRail(65,90,1);
        } else if (shLevel == DuckLine.SH_Levels.Collect){
            gotoHandRail(20,8,1);
        }
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
        this.gotoHandRail(0, point.handPercentage, point.handPower);
    }


    public double getRailPercent(){
        return (double)rail.getCurrentPosition() / railRange * 100;
    }

    public double getRailTicks() {
        return (double)rail.getCurrentPosition();
    }

    public int convRail_percent2ticks(double percent) {
        return (int)(percent / 100.0 * (double) railRange);
    }


    public double getHandPercent() {
        //                                               v might need to invert
        return ((double)(hand.getCurrentPosition() + potentiometer_offset) / handRange) * 100;
    }

    public double getHandTicks() {
        return (double)hand.getCurrentPosition();
    }

    public int convHand_percent2ticks(double percent) {
        return (int)(percent / 100.0 * (double)handRange) - potentiometer_offset;
    }


    public double getPotentiometerValue() {
        return potentiometer.getVoltage();
    }

    //updates
    public void telemetry_handRail() {
        opMode.telemetry.addData("hand:", "%3.2f%%, \t%d: ", getHandPercent(), this.hand.getCurrentPosition());
        opMode.telemetry.addData("rail: ","%3.2f%%, \t%d: ", getRailPercent(), this.rail.getCurrentPosition());
        opMode.telemetry.addData("potentiometer","offset: %d  %3.2fv:", potentiometer_offset, this.getPotentiometerValue());
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

    public void rail_drive(double power, boolean limitsOverride) {
        if(Math.abs(power) > 0.1 || rail.isBusy() == false) {
            setRailState(State.Drive);
            if (power > 0 && isRailBoundless(true, limitsOverride) &&  rail_limit_F.getState())
                rail.setPower(power);
            else if ( power < 0 && isRailBoundless(false, limitsOverride) && rail_limit_B.getState())
                rail.setPower(power);
            else
                rail.setPower(0);
        }
    }

    public void hand_drive(double power, boolean limitsOverride) {
        if(Math.abs(power) > 0.1 || hand.isBusy() == false) {
            setHandState(State.Drive);
            if(power > 0 && isHandBoundless(true, limitsOverride) && hand_limit_F.getState())
                hand.setPower(power);
            else if (power < 0 && isHandBoundless(false, limitsOverride) && hand_limit_B.getState())
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
            // updates  `
            stop(); // if touch switch is pressed then stop

        }*/
        return !grabber_switch.getState();
    }


    public void searchHome(){
        //repositioning hand...
//         while(this.getScaledPotentiometerValue() > 40) {
//            this.hand_drive(0.4, true);
//        }
//        this.hand_drive(0, true);
//        while(this.getScaledPotentiometerValue() < 60) {
//            this.hand_drive(0.3, true);
//        }
//
//        this.hand_drive(0, true);
        if(this.getHandPercent() > 80) {
            this.gotoHand(80, 0.8);
            //Timeout
            ElapsedTime timer = new ElapsedTime();
            while(hand.isBusy() && timer.seconds() < 1.75);
        }


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




    public int getScaledPotentiometerValue() {
        double pot_val = potentiometer.getVoltage();

        // map
        // NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
        // int tick = (int)(5000.0 * (1- pot_val / 3.33));

        double potVoltRange = 3.33;
        double potTickRange = 5000.0;
        double offset = ((double)handRange - potTickRange)/2;
        int tick = (int)(potTickRange * (1- pot_val / potVoltRange) + offset) ;
        return tick;
    }




    public void carouselRun(double power) {
        carousel.setPower(power);
    }

    public void carouselStop() {
        carousel.setPower(0);
    }

    public void switchSides(){
        setState(State.Goto);
        double pos = getRailPercent();
        if (pos < 50){
            gotoRail(100, 1);
        }
        else{
            gotoRail(0, 1);
        }
    }

    public boolean isBusy(float... timeout) {
        opMode.telemetry.addData( "Rail.isBusy: ",rail.isBusy());
        opMode.telemetry.addData( "Hand.isBusy: ", hand.isBusy());
        opMode.telemetry.update();

        ElapsedTime Timer = new ElapsedTime();

        if(timeout.length == 0) return (rail.isBusy() || hand.isBusy());
        else if(timeout.length == 1) return (rail.isBusy() || hand.isBusy()) && timeout[0] <= Timer.seconds();
        else
        return false;

    }

    public boolean isRailBoundless(boolean forward, boolean override) {
        return true;

        //returns true if the rail is allowed to move
//        if (!override) {
//            if (forward)
//                return (this.getHandPercent() > 10 || this.getRailPercent() < 20);
//            else
//                return (this.getHandPercent() < 90 || this.getRailPercent() > 80);
//        } else
//            return true;
    }

    public boolean isHandBoundless(boolean forward, boolean override) {
        return true;

        //returns true if the hand is allowed to move (to a certain side)
//        if (!override) {
//            if (forward)
//                return (this.getRailPercent() > 75 || this.getHandPercent() < 90);
//            else
//                return (this.getRailPercent() < 25 || this.getHandPercent() > 10);
//        } else
//            return true;
    }




}



