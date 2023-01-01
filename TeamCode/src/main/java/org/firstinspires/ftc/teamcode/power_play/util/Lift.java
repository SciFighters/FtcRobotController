package org.firstinspires.ftc.teamcode.power_play.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    public final int LIFT_RANGE = 3227, LIFT_MIN = 10; // max amount of ticks in the lift..
    public DcMotorEx rightElevator = null, leftElevator = null;
    public DigitalChannel touchDown = null;

    private Servo grabberRight = null, grabberLeft = null;


    public void init(HardwareMap hw) {
        touchDown = hw.get(DigitalChannel.class, "touchDown"); // Touch Sensor , bottom lift
        //region Set Elevator Motors
        rightElevator = hw.get(DcMotorEx.class, "RE");
        leftElevator = hw.get(DcMotorEx.class, "LE");
        rightElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        leftElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        rightElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftElevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //endregion
        //region Grabbers
        grabberRight = hw.get(Servo.class, "grabber_right");
        grabberLeft = hw.get(Servo.class, "grabber_left");
        grabberLeft.setDirection(Servo.Direction.FORWARD);
        grabberRight.setDirection(Servo.Direction.REVERSE);
        //endregion
        // resetLift();
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

    boolean elevatorTouchSwitch() {
        return !this.touchDown.getState();
    }


    public void grabber(boolean grab) {
        grabberLeft.setPosition(grab ? 1 : 0);
        grabberRight.setPosition(grab ? 1 : 0);
    }

    enum State {
        Maintain, // (pos: int)
        Manual,
        Goto,
    }

    State state;

    public enum LiftLevel {
        Floor(0),
        First(1264),
        Second(2066),
        Third(2845);

        final int position;

        LiftLevel(int p) {
            this.position = p;
        }
    }

    public void setLiftPower(double pow) {
        if (Math.abs(pow) > 0.25) {
            this.setState(State.Manual);
            rightElevator.setPower(pow);
            leftElevator.setPower(pow);
        } else if (!rightElevator.isBusy() || !leftElevator.isBusy()) { // Stick is 0 and right_elevator isn't busy.
            this.setState(State.Maintain); // Keep current position
        }
    }

    public void gotoLevel(LiftLevel level) {
        rightElevator.setTargetPosition(level.position);
        leftElevator.setTargetPosition(level.position);
        this.setState(State.Goto);
    }

    private void setState(State newState) {
        if (newState == this.state) return; // State unchanged => do nothing

        switch (newState) {
            case Manual:
                rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Run with power getting setPower from outside
                leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case Maintain:
                rightElevator.setTargetPosition(rightElevator.getCurrentPosition());
                rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightElevator.setPower(0.6);
                leftElevator.setTargetPosition(leftElevator.getCurrentPosition());
                leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftElevator.setPower(0.6);
                break;
            case Goto:
                rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightElevator.setPower(0.8);
                leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftElevator.setPower(0.8);
                break;
            default:
                break;
        }
        this.state = newState;
    }
}