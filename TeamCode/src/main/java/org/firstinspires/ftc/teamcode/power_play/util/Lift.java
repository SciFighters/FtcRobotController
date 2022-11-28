package org.firstinspires.ftc.teamcode.power_play.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    public final int LIFT_RANGE = 3227, LIFT_MIN = 10; // max amount of ticks in the lift..
    public DcMotorEx right_elevator = null;
    public DigitalChannel touchDown = null;
    public double startGoToX = 0; // use the relative position
    public double currentTarget = 0.001; // use to fix / goto position.
    final double defaultLiftPower = 0.7;

    private CRServo grabberRight = null, grabberLeft = null;

    boolean elevatorTouchSwitch() { return !this.touchDown.getState(); }

    private void resetLift() {
        if (this.elevatorTouchSwitch()) return;

        ElapsedTime timer = new ElapsedTime();

        right_elevator.setPower(-0.2);
        while (!this.elevatorTouchSwitch() && timer.seconds() < 4);
        right_elevator.setPower(0);

        right_elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getPos() {
        return (-this.right_elevator.getCurrentPosition());
    }

    public double getRelativePos(int ticks) {
        return (double) ticks / (double) LIFT_RANGE;
    }

    public double getRelativePos() {
        return getRelativePos(this.getPos());
    }

//    public void setTargetPos(int target) {
//        this.currentTarget = MathUtil.clamp(getRelativePos(target), 0, 1);
//    }
//
//
//    public void fixPos(int target) { // Target has to be provided as ticks, and is transferred to a relativePos
//        this.tickFixTarget = getRelativePos(target);
//        if (this.liftFixThread == null || this.liftFixThread.isAlive())
//            return; // Exits out of function
//        this.liftFixThread = new Thread() {
//            @Override
//            public void run() {
//                while (!MathUtil.inRange(getRelativePos(), tickFixTarget - 0.004, tickFixTarget + 0.004)) {
//                    setPower((tickFixTarget - getRelativePos()));
//                }
//                try {
//                    this.join();
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }
//        };
//        this.liftFixThread.start();
//    }

    public void init(HardwareMap hw) {
        touchDown = hw.get(DigitalChannel.class, "touchDown"); // Touch Sensor , bottom lift

        right_elevator = hw.get(DcMotorEx.class, "RE");
        right_elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right_elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        grabberRight = hw.get(CRServo.class, "grabber_right");
        grabberLeft = hw.get(CRServo.class, "grabber_left");

        grabberLeft.setDirection(CRServo.Direction.FORWARD);
        grabberRight.setDirection(CRServo.Direction.REVERSE);

//        resetLift();
    }

    public void setGrabbersPower(double pow) {
        grabberLeft.setPower(pow);
        grabberRight.setPower(pow);
    }

//    public void goToEdge(double maxPower, boolean restartGoToXsetLastPos, LiftDirection direction) { // maxPower is the target-maximum power of the lift. ResetLastPos - to reset or not the lastPosition (if the movement starts anew, it should be reset). Direction - the direction in which the lift is traveling
//        lastMovePos = resetLastPos ? getRelativePos() : lastMovePos;
//        this.setPower(direction.directionMul * 0.05 +
//                (direction.directionMul * 0.95 * maxPower) * ((Math.abs(this.getRelativePos() - this.lastMovePos) /
//                (direction.targetValue - direction.directionMul * this.lastMovePos))));
//    }
//
//    public void goToEdge(double maxPower, LiftDirection direction) { this.goToEdge(maxPower, false, direction); }
//
//    public void goTo(double maxPower, boolean resetPos, double targetX) {
//        if (MathUtil.inRange(targetX, this.getRelativePos() - 0.03, this.getRelativePos() + 0.03)) {
//            startGoToX = resetPos ? getRelativePos() : startGoToX;
//            this.setPower(0.1 + (maxPower * (0.9 - 0.001)) *
//                    (((getRelativePos() - startGoToX) / (targetX - startGoToX)) >= (0.5 - 0.01) ?
//                            (((getRelativePos() - ) / (targetX - startGoToX)) * 2) :
//                            (1 + 1 - 2 * ((getRelativePos() - startGoToX) / (targetX - startGoToX))))
//            );
//        } else this.setPower(0);
//    }
//
//    public void goTo(double maxPower) { // CAN BE USED AS FIX POSITION AS WELL (DON'T CHANGE OR REMOVE)
//        if (MathUtil.inRange(this.getRelativePos(), this.currentTarget - 10, this.currentTarget + 10)) {
//            this.setPower(0);
//        } else if (MathUtil.outOfRange(this.getRelativePos(), this.currentTarget - 7, this.currentTarget + 7)) {
//            this.setPower(maxPower * Math.signum(this.currentTarget - this.getRelativePos()));
//        } else {
//            this.setPower(0.2 * Math.signum(this.currentTarget - this.getRelativePos()));
//        }
//    }
//
//    public void goTo() {
//        this.goTo(this.defaultLiftPower);
//    }
//
//    public void setPower(double... power) {
//        if ((!(this.getPos() < this.LIFT_MIN && getAveragePower(power) < 0)) ||
//                (!(this.getPos() > this.LIFT_RANGE && getAveragePower(power) > 0))) {
//            RE.setPower(power[0]);
//            if (power.length == 2) {
////            LL.setPower(power[1]);
//                return;
//            }
////        LL.setPower(power[0]);
//            doNothing();
//        } else {
//            RE.setPower(0);
//        }
//    }
    public void setPower(double pow) {
        right_elevator.setPower(this.getPos() >= this.LIFT_RANGE - 90 ? -0.1 : pow);
    }

    public double getAveragePower(double[] power) {
        double avg = 0;
        for (double v : power) {
            avg += v;
        }
        return avg / power.length;
    }

    public double getAveragePower() {
        return this.getAveragePower(this.getPower());
    }

    public void breakMotor() {

        right_elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        right_elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double[] getPower() {
        return new double[]{this.right_elevator.getPower()};
    }

    public static void doNothing() {
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