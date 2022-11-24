package org.firstinspires.ftc.teamcode.power_play.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    public final int LIFT_RANGE = 3227, LIFT_MIN = 10; // max amount of ticks in the lift..
    public DcMotorEx right_elevator = null;
    public DigitalChannel touchDown = null;

    private CRServo grabberRight = null, grabberLeft = null;


    public void init(HardwareMap hw) {
        touchDown = hw.get(DigitalChannel.class, "touchDown"); // Touch Sensor , bottom lift

        right_elevator = hw.get(DcMotorEx.class, "RE");
        right_elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right_elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        right_elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        grabberRight = hw.get(CRServo.class, "grabber_right");
        grabberLeft = hw.get(CRServo.class, "grabber_left");

        grabberLeft.setDirection(CRServo.Direction.FORWARD);
        grabberRight.setDirection(CRServo.Direction.REVERSE);

        // resetLift();
    }

    private void resetLift() {
        if (this.elevatorTouchSwitch()) return;

        ElapsedTime timer = new ElapsedTime();

        right_elevator.setPower(-0.2);
        while (!this.elevatorTouchSwitch() && timer.seconds() < 4);
        right_elevator.setPower(0);

        right_elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    boolean elevatorTouchSwitch() { return !this.touchDown.getState(); }


    public void setGrabbersPower(double pow) {
        grabberLeft.setPower(pow);
        grabberRight.setPower(pow);
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
        LiftLevel(int p) { this.position = p; }
    }

    public void setLiftPower(double pow) {
        if (Math.abs(pow) > 0.25) {
            this.setState(State.Manual);
            right_elevator.setPower(pow);
        } else {
            if (!right_elevator.isBusy()) {
                this.setState(State.Maintain);
                right_elevator.setPower(0.8);
            }
        }
    }

    public void gotoLevel(LiftLevel level) {
        right_elevator.setTargetPosition(level.position);
        this.setState(State.Goto);
    }

    private void setState(State newState) {
        if (newState == this.state) return;

        if (newState == State.Manual) {
            right_elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.state = State.Manual;
        } else if (newState == State.Maintain) {
            right_elevator.setTargetPosition(right_elevator.getCurrentPosition());
            right_elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_elevator.setPower(0.8);
            this.state = State.Maintain;
        } else if (newState == State.Goto) {
            right_elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_elevator.setPower(0.8);
            this.state = State.Goto;
        }
    }
}