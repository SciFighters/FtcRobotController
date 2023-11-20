package org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.States.GoToState;
import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.States.GraceHoldTimeState;
import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.States.HoldState;
import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.States.IdleState;
import org.firstinspires.ftc.teamcode.centerstage.ArmPIDTesting.States.ManualState;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;

import java.util.Objects;

public class Arm {
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
    private DcMotorEx carousel = null;
    private final int handMin = -1200;
    private final int handMax = 0;
    private final LinearOpMode opMode;
    private final double deadZone = 0.05;
    private final double holdPower = 0.2;
    int handTargetPos, railTargetPos;
    public ElapsedTime timer;
    public double graceTimeLimit = 0.25;
    public State<Arm> gotoState, graceHoldTimeState, holdState, idleState, manualState;

    public double getHoldPower() {
        return holdPower;
    }


    public enum Position {
        One(-500, 2), Two(-900, 2), Three(-1200, 2); // Todo : put actual values
        final int handPos, railPos;

        Position(int hanPos, int railPos) {
            this.handPos = hanPos;
            this.railPos = railPos;
        }
    }

    public StateMachine<Arm> stateMachine;

    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init(HardwareMap hw) {
        gotoState = new GoToState();
        graceHoldTimeState = new GraceHoldTimeState();
        holdState = new HoldState();
        idleState = new IdleState();
        manualState = new ManualState();
        timer = new ElapsedTime();
        stateMachine = new StateMachine<>(this);
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
        stateMachine.changeState(idleState);


//        hand.setTargetPositionTolerance(60);
//        rail.setTargetPositionTolerance(50);

        // done inside searchHome()  // resetPotAndHand();
        //searchHome();
    }

    public void goToPos(Position pos) {
        setTargetPositions(pos.handPos, pos.railPos);
        stateMachine.changeState(gotoState);
    }

    public int getCurrentHandPos() {
        return hand.getCurrentPosition();
    }

    public int getCurrentRailPos() {
        return rail.getCurrentPosition();
    }


    public void setTargetPositions(int handPos, int railPos) {
        handTargetPos = handPos;
        railTargetPos = railPos;
    }

    public void setManualMode(boolean manual, double power) {
        if (manual) {
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
        stateMachine.execute();
    }

    public boolean areMotorsBusy() {
        return !hand.isBusy() && !rail.isBusy();
    }

    public void setArmPower(double railPower, double handPower) {
        if (outOfDeadZone(handPower)) {
            stateMachine.changeState(manualState);
            handSetPower(handPower);
        } else {
            if (stateMachine.getCurrentState() != gotoState)
                stateMachine.changeState(holdState);
        }

        handleStates();
    }

    public void handSetPower(double handPower) {
        hand.setPower(handPower);
    }

    public boolean outOfDeadZone(double power) {
        return Math.abs(power) > deadZone;
    }

    public int getHandPos() {
        return hand.getCurrentPosition();
    }
}
