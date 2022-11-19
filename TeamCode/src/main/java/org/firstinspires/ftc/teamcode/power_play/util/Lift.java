package org.firstinspires.ftc.teamcode.power_play.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;

public class Lift {
    // Declaring variables
//    private OpMode opMode;
    public final int LIFT_RANGE = 3227, LIFT_MIN = 10; // max amount of ticks in the lift..
    //  public DcMotor RE, LE;
    public DcMotorEx RE;
    public double startGoToX = 0; // use the relative position
    public double currentTarget = 0.001; // use to fix / goto position.
    final double defaultLiftPower = 0.7;

//    private Thread liftFixThread;
    volatile double tickFixTarget = 0;

    private CRServo grabberRight = null, grabberLeft = null;

    public enum Levels {
        lvl1(297, 'a'),
        lvl2(1264, 'b'),
        lvl3(2066, 'x'),
        lvl4(2845, 'y');
        // Declaring attributes
        public int ticks;
        public Toggle toggle;
        public char button;

        Levels(int ticks, char button) { // Constructor
            this.ticks = ticks;
            this.button = button;
            this.toggle = new Toggle();
        }

        public void update(OpMode opMode) {
            switch (this.button) {
                case 'a':
                    this.toggle.update(opMode.gamepad1.a);
                    break;
                case 'b':
                    this.toggle.update(opMode.gamepad1.b);
                    break;
                case 'x':
                    this.toggle.update(opMode.gamepad1.x);
                    break;
                case 'y':
                    this.toggle.update(opMode.gamepad1.y);
                    break;
                default:
                    this.toggle.update(false);
                    break;
            }
        }

        public boolean isPressed() {
            return this.toggle.isPressed();
        }

        public boolean isClicked() {
            return this.toggle.isClicked();
        }
    }

    public void setTargetPos(int target) {
        this.currentTarget = MathUtil.clamp(getRelativePos(target), 0, 1);
    }

    public double getRelativePos(int ticks) {
        return (double) ticks / (double) LIFT_RANGE;
    }

    public double getRelativePos() {
        return getRelativePos(this.getPos());
    }

    public int getPos() {
        return (-this.RE.getCurrentPosition());
    }

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
        RE = hw.get(DcMotorEx.class, "RE");
//        LL = hw.get(DcMotor.class, "LL");
        RE.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RE.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        RE.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        grabberRight = hw.get(CRServo.class, "grabber_right");
        grabberLeft = hw.get(CRServo.class, "grabber_left");

        grabberLeft.setDirection(CRServo.Direction.FORWARD);
        grabberRight.setDirection(CRServo.Direction.REVERSE);
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

    public void goTo(double maxPower) { // CAN BE USED AS FIX POSITION AS WELL (DON'T CHANGE OR REMOVE)
        if (MathUtil.inRange(this.getRelativePos(),     this.currentTarget - 0.01, this.currentTarget + 0.01)) {
//            this.startGoToX = resetPos ? getRelativePos() : startGoToX;
            this.setPower(0);
        } else if(MathUtil.outOfRange(this.getRelativePos(),  this.currentTarget - 0.018, this.currentTarget + 0.018)) {
            this.setPower(maxPower * Math.signum(this.currentTarget - this.getRelativePos()));
        } else {
            this.setPower(0.2 * Math.signum(this.currentTarget - this.getRelativePos()));
        }
    }

    public void goTo() {
        this.goTo(this.defaultLiftPower);
    }

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
        RE.setPower(this.getPos() >= this.LIFT_RANGE - 90 ? -0.1 : pow);
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

        RE.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void reset() {
        RE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double[] getPower() {
        return new double[]{this.RE.getPower()};
    }

    public static void doNothing() {
    }
}