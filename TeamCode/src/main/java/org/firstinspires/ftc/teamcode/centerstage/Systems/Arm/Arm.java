package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.GoToState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.GraceHoldTimeState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.HoldState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.IdleState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.ManualState;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.StateMachine;

public class Arm {
    DcMotorEx motor1 = null, motor2 = null;

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
    private final int minPos = -1200;
    private final int maxPos = 0;
    private final LinearOpMode opMode;
    private final double deadZone = 0.05;
    private final double holdPower = 0.2;
    int targetPos;
    public ElapsedTime timer;
    public double graceTimeLimit = 0.25;
    public State<Arm> gotoState, graceHoldTimeState, holdState, idleState, manualState;

    public double getHoldPower() {
        return holdPower;
    }


    public enum Position {
        One(-500), Two(-900), Three(-1200); // Todo : put actual values
        final int liftPos;

        Position(int liftPos) {
            this.liftPos = liftPos;
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
        motor1 = hw.get(DcMotorEx.class, "armMotor1");
        motor2 = hw.get(DcMotorEx.class, "armMotor2");
        motor1.setDirection(DcMotorEx.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        grabbers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        grabber_right = hw.get(CRServo.class, "grabber_right");
//        grabber_left = hw.get(CRServo.class, "grabber_left");
////        grabbers = hw.get(DcMotor.class, "grabbers");
//
//        capping_servo = hw.get(Servo.class, "capping_servo");
//
//        // Setting directions
//        grabber_left.setDirection(CRServo.Direction.FORWARD);
//        grabber_right.setDirection(CRServo.Direction.REVERSE);
//        capping_servo.setDirection(Servo.Direction.FORWARD);
//        capping_servo.scaleRange(0.0, 1.0);
//
//        grabber_switch = hw.get(DigitalChannel.class, "grabber_switch");
////       hand_limit = hw.get(DigitalChannel.class, "hand_limit_front");
//
//        potentiometer = hw.get(AnalogInput.class, "potentiometer");
        stateMachine.changeState(idleState);


//        motor1.setTargetPositionTolerance(60);
//        rail.setTargetPositionTolerance(50);

        // done inside searchHome()  // resetPotAndHand();
        //searchHome();
    }

    public void goToPos(Position pos) {
        setTargetPositions(pos.liftPos);
        stateMachine.changeState(gotoState);
    }

    public int getPos() {
        return motor1.getCurrentPosition();
    }

    public void setTargetPositions(int pos) {
        targetPos = pos;
    }

    public void setManualMode(boolean manual, double power) {
        if (manual) {
            motor1.setPower(0);
            motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            motor2.setPower(0);
            motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            motor1.setTargetPosition(targetPos);
            motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor1.setPower(power);

            motor2.setTargetPosition(targetPos);
            motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor2.setPower(power);
        }
    }

    public void handleStates() {
        stateMachine.execute();
    }

    public boolean areMotorsBusy() {
        return !motor1.isBusy() && !motor2.isBusy();
    }

    public void setPower(double power) {
        if (outOfDeadZone(power)) {
            stateMachine.changeState(manualState);
            setMotorsPower(power);
        } else {
            if (stateMachine.getCurrentState() != gotoState)
                stateMachine.changeState(holdState);
        }

        handleStates();
    }

    public void setMotorsPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public boolean outOfDeadZone(double power) {
        return Math.abs(power) > deadZone;
    }

}
