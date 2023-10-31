package org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends Thread {
    DcMotorEx rail = null, hand = null;

    private CRServo grabber_right = null;
    private CRServo grabber_left = null;
    //private DcMotor grabbers = null;

    private Servo capping_servo = null; // Shipping elements servo
    private DigitalChannel grabber_switch = null;
    private DigitalChannel hand_limit_B = null;
    private DigitalChannel hand_limit_F = null;
    private DigitalChannel rail_limit_B = null;
    private DigitalChannel rail_limit_F = null;

    private AnalogInput potentiometer = null;
    private int potentiometer_offset;

    private DcMotorEx carousel = null;

    private int railRange = 1470;
    public int handRange = 3935; // 6000; // 5968;
    private LinearOpMode opMode;
    private double deadZone = 0.05;
    volatile private double power = 1;
    int handTargetPos, railTargetPos;

    public enum State {
        Idle, Hold, Manual, Goto,
    }

    public enum Position {
        One(1, 2), Two(2, 2), Three(3, 2); // Todo : put actual values
        final int handPos, railPos;

        Position(int hanPos, int railPos) {
            this.handPos = hanPos;
            this.railPos = railPos;
        }
    }

    State state;

    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        opMode.telemetry.addLine("Head Rail Init");
        rail = hw.get(DcMotorEx.class, "rail");// Getting from hardware map
        hand = hw.get(DcMotorEx.class, "hand");

        rail.setDirection(DcMotorEx.Direction.REVERSE);// Setting directions
        hand.setDirection(DcMotorEx.Direction.FORWARD);

        rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);// Setting encoders
        hand.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        grabbers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabber_right = hw.get(CRServo.class, "grabber_right");
        grabber_left = hw.get(CRServo.class, "grabber_left");
//        grabbers = hw.get(DcMotor.class, "grabbers");

        capping_servo = hw.get(Servo.class, "capping_servo");

        // Setting directions
        grabber_left.setDirection(CRServo.Direction.FORWARD);
        grabber_right.setDirection(CRServo.Direction.REVERSE);
        capping_servo.setDirection(Servo.Direction.FORWARD);
        capping_servo.scaleRange(0.0, 1.0);

        grabber_switch = hw.get(DigitalChannel.class, "grabber_switch");
//       hand_limit = hw.get(DigitalChannel.class, "hand_limit_front");
        rail_limit_B = hw.get(DigitalChannel.class, "rail_limit_back");
        rail_limit_F = hw.get(DigitalChannel.class, "rail_limit_front");

        hand_limit_B = hw.get(DigitalChannel.class, "hand_limit_back");
        hand_limit_F = hw.get(DigitalChannel.class, "hand_limit_front");

        carousel = hw.get(DcMotorEx.class, "carousel");
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        potentiometer = hw.get(AnalogInput.class, "potentiometer");


//        hand.setTargetPositionTolerance(60);
//        rail.setTargetPositionTolerance(50);

        // done inside searchHome()  // resetPotAndHand();
        //searchHome();
    }

    public void goToPos(Position pos) {
        setTargetPositions(pos.handPos, pos.railPos);
        setState(State.Goto);
    }

    public void setState(State newState) {
        if (newState == state) return;

        switch (newState) {
            case Idle: {

                break;
            }
            case Manual: {
                // Controller
                setArmDriveMode(true);
                break;
            }
            case Hold: {
                setTargetPositions(hand.getCurrentPosition(), rail.getCurrentPosition());
                setArmDriveMode(false);
                break;
            }
            case Goto: {
                setArmDriveMode(false);
                break;
            }
        }

        this.state = newState;
    }

    public void setTargetPositions(int handPos, int railPos) {
        handTargetPos = handPos;
        railTargetPos = railPos;
    }

    public void setArmDriveMode(boolean drive) {
        if (drive) {
            hand.setPower(0);
            hand.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rail.setPower(0);
            rail.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            hand.setTargetPosition(handTargetPos);
            hand.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            hand.setPower(power);
            rail.setTargetPosition(railTargetPos);
            rail.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rail.setPower(power / 2);
        }
    }

    public void handleStates() {
        switch (state) {
            case Hold: {
                if (areMotorsBusy()) {
                    setArmDriveMode(false);
                }
                break;
            }
            case Goto: {
                if (areMotorsBusy()) {
                    setState(State.Hold);
                }
            }
        }
    }

    public boolean areMotorsBusy() {
        return !hand.isBusy() && !rail.isBusy();
    }

    public void setArmPower(double railPower, double handPower) {
        if (outOfDeadZone(handPower)) {
            setState(State.Manual);
            handSetPower(handPower);
        } else {
            setState(State.Hold);
        }

        handleStates();
    }

    public void handSetPower(double handPower) {
        hand.setPower(handPower);
    }

    public boolean outOfDeadZone(double power) {
        return Math.abs(power) > deadZone;
    }
}
