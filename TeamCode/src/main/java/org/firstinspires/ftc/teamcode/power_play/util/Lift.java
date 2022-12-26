package org.firstinspires.ftc.teamcode.power_play.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//DONT DELETE 1194 2013
public class Lift {
    public final int LIFT_RANGE = 1150, LIFT_MIN = 10; // max amount of ticks in the lift..
    public final int FLIP_POSITION = 120, HALF_POSITION = 60; //flip motor max count of 180 degrees
    public DcMotorEx rightElevator = null, leftElevator = null;
    public DigitalChannel touchDown = null;
    public DigitalChannel flipTouchSwitch = null;
    public DcMotor flipMotor = null;
    private Servo grabberRight = null, grabberLeft = null, rotateServo = null;


    public void init(HardwareMap hw) {
        flipMotor = hw.get(DcMotor.class, "JM");
        touchDown = hw.get(DigitalChannel.class, "touchDown"); // Touch Sensor , bottom lift
        //region Set Elevator Motors
        rightElevator = hw.get(DcMotorEx.class, "RE");
        leftElevator = hw.get(DcMotorEx.class, "LE");
        rightElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rightElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightElevator.setDirection(DcMotorEx.Direction.REVERSE);
        leftElevator.setDirection(DcMotorEx.Direction.FORWARD);

        rightElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flipMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int gotoTolerance = 32;
        rightElevator.setTargetPositionTolerance(gotoTolerance);
        leftElevator.setTargetPositionTolerance(gotoTolerance);

        //endregion
        //region Gathering System
        grabberRight = hw.get(Servo.class, "grabber_right");
        grabberLeft = hw.get(Servo.class, "grabber_left");
        grabberLeft.setDirection(Servo.Direction.FORWARD);
        grabberRight.setDirection(Servo.Direction.REVERSE);
        flipTouchSwitch = hw.get(DigitalChannel.class, "JT");
        rotateServo = hw.get(Servo.class, "rotateServo");
        //endregion
        //resetLift();

        resetJoint();
    }

    private void resetLift() {
        if (this.elevatorTouchSwitch()) return;

        ElapsedTime timer = new ElapsedTime();

        rightElevator.setPower(-0.2);
        leftElevator.setPower(-0.2);
        while (!this.elevatorTouchSwitch() && timer.seconds() < 4) ;
        rightElevator.setPower(0);
        leftElevator.setPower(0);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetJoint() {
        if (this.jointTouchSwitch()) return;

        grabber(true);
        // ElapsedTime timer = new ElapsedTime(); // not sure why exists

        flipMotor.setPower(-0.2);
        while (!this.jointTouchSwitch() /*&& timer.seconds() < 4*/);
        flipMotor.setPower(0);
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    boolean elevatorTouchSwitch() {
        return !this.touchDown.getState();
    }

    boolean jointTouchSwitch() {
        return !this.flipTouchSwitch.getState();
    }

    public void grabber(boolean grab) {
        grabberLeft.setPosition(grab ? 1 : 0);
        grabberRight.setPosition(grab ? 1 : 0);
    }


    public void rotate(boolean rotated) {
        rotateServo.setPosition(rotated ? 1 : 0);
    }

    public void toggleFlip() {
        if (this.armState != ArmState.Flip) setArmState(ArmState.Flip);
        else setArmState(ArmState.Home);
    }

    public enum LiftState {
        Idle,
        Maintain, // (pos: int),
        Manual,
        Goto;
    }

    private LiftState liftState;

    public LiftState getState() {
        return liftState;
    }


    public enum ArmState {
        Home,
        Flip,
        Half,
        Begin,
        Rotate1,
        Rotate2,
        End
    }

    ArmState armState;

    public enum LiftLevel {
        Floor(0), First(292), Second(601), Third(886);

        final int position;

        LiftLevel(int p) {
            this.position = p;
        }
    }

    public void setLiftPower(double pow) {
        if (Math.abs(pow) > 0.2) {
            if (pow < 0) { // If negative
                pow /= 3;
                if (leftElevator.getCurrentPosition() > 400)
                    pow = 0.3 * (double) (leftElevator.getCurrentPosition() - 400) / LIFT_RANGE + pow / 4;
//                if (leftElevator.getCurrentPosition() > 400) {
//                    pow = 0.1 * (double) (leftElevator.getCurrentPosition() - 400) / LIFT_RANGE;
//                } else {
//                    pow = 0.3 * (double) (leftElevator.getCurrentPosition() - 400) / LIFT_RANGE + pow / 3;
//                }
            }
            this.setLiftState(LiftState.Manual);
            rightElevator.setPower(pow);
            leftElevator.setPower(pow);
        } else if (!leftElevator.isBusy() && !(liftState == LiftState.Idle)) { // Stick is 0 and right_elevator isn't busy.
            this.setLiftState(LiftState.Maintain); // Keep current position
        }
    }


//    public void update() {
////        if (!rightElevator.isBusy() || !leftElevator.isBusy()) { // Stick is 0 and right_elevator isn't busy.
////            this.setLiftState(LiftState.Maintain); // Keep current position
////        }
//        switch (this.armState) {
//            case Begin:
//                if (!this.flipMotor.isBusy()) this.setArmState(ArmState.Rotate1);
//                break;
//            case Rotate1:
//
//                break;
//            case Flip:
//                if (!this.flipMotor.isBusy()) this.setArmState(ArmState.Rotate2);
//                break;
//            case Rotate2:
//
//                break;
//            default:
//                break;
//        }
//    }

    public void gotoLevel(LiftLevel level) {
        if (level == LiftLevel.Floor) setArmState(ArmState.Home);
        else setArmState(ArmState.Flip); // Not sure what does half mean, changed to flip
        rightElevator.setTargetPosition(level.position);
        leftElevator.setTargetPosition(level.position);
        this.setLiftState(LiftState.Goto);
    }

    public void setLiftState(LiftState newLiftState) {
        if (newLiftState == this.liftState) return; // State unchanged => do nothing

        switch (newLiftState) {
            case Idle:
                rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightElevator.setPower(0);
                leftElevator.setPower(0);
                flipMotor.setPower(0);
                break;
            case Manual:
                rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Run with power getting setPower from outside
                leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case Maintain:
                int t = leftElevator.getCurrentPosition();
                rightElevator.setTargetPosition(t);
                rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightElevator.setPower(0.3);
                leftElevator.setTargetPosition(t);
                leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftElevator.setPower(0.3);
                break;
            case Goto:
                rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightElevator.setPower(1);
                leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftElevator.setPower(1);
                break;
            default:
                break;
        }
        this.liftState = newLiftState;
    }

    public void setArmState(ArmState newState) {
        if (newState == this.armState) return;
        switch (newState) {
            case Home:
                flipMotor.setTargetPosition(0);
                flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipMotor.setPower(0.7);
                rotateServo.setPosition(0);
                grabber(true);
                break;
            case Flip:
                flipMotor.setTargetPosition(FLIP_POSITION);
                flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipMotor.setPower(0.8);
                rotateServo.setPosition(1);
                break;
            case Half:
                flipMotor.setTargetPosition(HALF_POSITION);
                flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipMotor.setPower(0.8);
                rotateServo.setPosition(1);
                break;
            case Begin:
                flipMotor.setTargetPosition(30);
                flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                flipMotor.setPower(0.8);
                break;
            case Rotate1:
                rotateServo.setPosition(0.5);
                break;

            case Rotate2:
                rotateServo.setPosition(1);
                break;
            case End:
                break;
            default:
                break;
        }
        this.armState = newState;
    }

    public ArmState getArmState() {
        return this.armState;
    }
}