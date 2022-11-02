package org.firstinspires.ftc.teamcode.power_play.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    // Declaring variables
//    private OpMode opMode;
    public final int LiftRange = 1000; // max amount of ticks in the lift..
//    DcMotor RL, LL;
    public DcMotor RL;
    public double lastMovePos = 0; // use the relative position


    enum LiftDirection {
        TOP(1, 1),
        BOTTOM(-1, 0);
        double directionMul, targetValue;
        LiftDirection(double directionMul, double targetValue) {
            this.directionMul = directionMul;
            this.targetValue = targetValue;
        }
    }

    public double getRelativePos(int ticks) { return (double)ticks / (double)LiftRange; }
    public double getRelativePos() {return getRelativePos(this.getPos()); }
    public int getPos() { return (this.RL.getCurrentPosition()); }
    public void init(HardwareMap hw) {
        RL = hw.get(DcMotor.class, "RL");
//        LL = hw.get(DcMotor.class, "LL");
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void goToEdge(double maxPower, boolean resetLastPos, LiftDirection direction) { // maxPower is the target-maximum power of the lift. ResetLastPos - to reset or not the lastPosition (if the movement starts anew, it should be reset). Direction - the direction in which the lift is traveling
        lastMovePos = resetLastPos ? getRelativePos() : lastMovePos;
        this.setPower(direction.directionMul * 0.05 +
                (direction.directionMul * 0.95 * maxPower) *
                (Math.abs(this.getRelativePos() - this.lastMovePos) /
                (direction.targetValue - direction.directionMul * this.lastMovePos)) * (
                1 - (Math.abs(this.getRelativePos() - this.lastMovePos) /
                (direction.targetValue - direction.directionMul * this.lastMovePos))));
    }

    public void goToEdge(double maxPower, LiftDirection direction) { this.goToEdge(maxPower, false, direction); }

    public void setPower(double... power) {
        RL.setPower(power[0]);
        if(power.length == 2) {
//            LL.setPower(power[1]);
            return;
        }
//        LL.setPower(power[0]);
        doNothing();
    }
    public static void doNothing() {}
}