package org.firstinspires.ftc.teamcode.power_play.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    // Declaring variables
//    private OpMode opMode;
    public final int LiftRange = 1000, LiftMin = 10; // max amount of ticks in the lift..
    //  public DcMotor RE, LE;
    public DcMotorEx RE;
    public double startGoToX = 0; // use the relative position


    public enum LiftDirection {
        TOP(1, 1),
        BOTTOM(-1, 0);
        double directionMul, targetValue;

        LiftDirection(double directionMul, double targetValue) {
            this.directionMul = directionMul;
            this.targetValue = targetValue;
        }
    }

    public double getRelativePos(int ticks) {
        return (double) ticks / (double) LiftRange;
    }

    public double getRelativePos() {
        return getRelativePos(this.getPos());
    }

    public int getPos() {
        return (this.RE.getCurrentPosition());
    }

    public void init(HardwareMap hw) {
        RE = hw.get(DcMotorEx.class, "RE");
//        LL = hw.get(DcMotor.class, "LL");
        RE.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RE.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

//    public void goToEdge(double maxPower, boolean resetLastPos, LiftDirection direction) { // maxPower is the target-maximum power of the lift. ResetLastPos - to reset or not the lastPosition (if the movement starts anew, it should be reset). Direction - the direction in which the lift is traveling
//        lastMovePos = resetLastPos ? getRelativePos() : lastMovePos;
//        this.setPower(direction.directionMul * 0.05 +
//                (direction.directionMul * 0.95 * maxPower) * ((Math.abs(this.getRelativePos() - this.lastMovePos) /
//                (direction.targetValue - direction.directionMul * this.lastMovePos))));
//    }
//
//    public void goToEdge(double maxPower, LiftDirection direction) { this.goToEdge(maxPower, false, direction); }

    public void goTo(double maxPower, boolean resetPos, double targetX) {
        if (inRange(targetX, this.getRelativePos() - 0.03, this.getRelativePos() + 0.03)) {
            startGoToX = resetPos ? getRelativePos() : startGoToX;
            this.setPower(0.1 + (maxPower * (0.9 - 0.001)) *
                    (((getRelativePos() - startGoToX) / (targetX - startGoToX)) >= (0.5 - 0.01) ?
                            (((getRelativePos() - startGoToX) / (targetX - startGoToX)) * 2) :
                            (1 + 1 - 2 * ((getRelativePos() - startGoToX) / (targetX - startGoToX))))
            );
        } else this.setPower(0);
    }

    private boolean inRange(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value > min && value < max;
    }


//    private boolean approximately(double value, double valuedAt, double approximation) {
//        return this.inRange(value, valuedAt - approximation, valuedAt + approximation);
//    }

    public void goTo(boolean resetPos, double targetX) {
        this.goTo(0.99, resetPos, targetX);
    }

    public void setPower(double... power) {
        if (inRange(this.LiftMin, this.getPos(), this.LiftRange)) {
            RE.setPower(power[0]);
            if (power.length == 2) {
//            LL.setPower(power[1]);
                return;
            }
//        LL.setPower(power[0]);
            doNothing();
        } else {
            RE.setPower(0);
        }
    }
    public void breakMotor (){
       RE.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public double[] getPower() {
        return new double[]{this.RE.getPower()};
    }

    public static void doNothing() {
    }
}