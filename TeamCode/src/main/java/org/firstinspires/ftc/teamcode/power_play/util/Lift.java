package org.firstinspires.ftc.teamcode.power_play.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//DON'T DELETE 1194 2013
public class Lift {
    //region Vars
    public final double liftDeadZone = 0.05;
    private double prevArmPos = 0.5;
    public final int MIN_HEIGHT_TO_ARM_MOVEMENT = 1120; // Minimal elevator height to make arm (horizontal) movement
    public static final int LIFT_RANGE = 3050, LIFT_MIN = 10; // max amount of ticks in the lift..
    public final int FLIP_POSITION = 139; //flip motor max count of 180 degrees
    public final int FLIP_TOLERANCE = 40;
    public int liftDescentLevel = 0;
    public LiftLevel currentLevelTarget = null;
    //endregion
    //region Referencesa
    private LinearOpMode opMode;

    public DcMotorEx rightElevator = null, leftElevator = null;
    public DcMotorEx jointMotor = null; // The Flip Motor

    public DigitalChannel touchDown = null;
//    public DigitalChannel flipTouchSwitch = null;
//    public DigitalChannel flipTouchSwitch2 = null;

    private Servo grabberRight = null, grabberLeft = null /*, rotateServo = null*/;
    public Servo armServoLeft, armServoRight, camServo;
// endregion

    public void init(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
//        jointMotor = hw.get(DcMotorEx.class, "JM"); // The Flip Motor
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

//        jointMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        jointMotor.setDirection(DcMotor.Direction.REVERSE);
//        jointMotor.setTargetPositionTolerance(25);

        final int gotoTolerance = 10;
        rightElevator.setTargetPositionTolerance(gotoTolerance);
        leftElevator.setTargetPositionTolerance(gotoTolerance);

        //endregion
        //region Gathering System
        grabberRight = hw.get(Servo.class, "grabber_right");
        grabberLeft = hw.get(Servo.class, "grabber_left");
        grabberLeft.setDirection(Servo.Direction.FORWARD);
        grabberRight.setDirection(Servo.Direction.REVERSE);
//        flipTouchSwitch = hw.get(DigitalChannel.class, "JT");
//        flipTouchSwitch2 = hw.get(DigitalChannel.class, "JT2");
//        rotateServo = hw.get(Servo.class, "rotateServo");
        armServoLeft = hw.get(Servo.class, "servoArmLeft");
        armServoRight = hw.get(Servo.class, "servoArmRight");
//        camServo = hw.get(Servo.class, "CamServo");
        //endregion
        ResetArmPos();
        grabber(true);
        resetLift();
        resetJoint();
    }

    public void resetLift() {
        ResetArmPos();
        opMode.sleep(300);
        opMode.telemetry.addData("Resetting Lift...", "");
        opMode.telemetry.update();
        if (this.elevatorTouchSwitch()) return;
        ResetArmPos();
        ElapsedTime timer = new ElapsedTime();

        rightElevator.setPower(-0.3);
        leftElevator.setPower(-0.3);
        while ((!this.elevatorTouchSwitch()) && timer.seconds() < 2) ;
        opMode.telemetry.addData("Touch switch, clicked", true);
        opMode.telemetry.update();
        rightElevator.setPower(0);
        leftElevator.setPower(0);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    public void MoveCameraServo(double pos) {
//        camServo.setPosition(pos);
//    }

    public void MoveCameraServo(boolean open) {
//        camServo.setPosition(open ? 0 : 1);
    }

//    public double GetCameraServoPos() {
////        return camServo.getPosition();
//    }

    public void SaveArmPos() {
        prevArmPos = getArmPos();
    }

    public void ResetArmPos() {
        armGotoPosition(0.5, false);
        prevArmPos = 0.5;
    }

    public void resetJoint() {
        opMode.telemetry.addData("Resetting Joint...", "");
        opMode.telemetry.update();
//        if (this.jointTouchSwitch()) return;

        grabber(false);
//        ElapsedTime timer = new ElapsedTime();
//        jointMotor.setPower(-0.2);
//        while (!this.jointTouchSwitch() && timer.seconds() < 1) ;
//        jointMotor.setPower(0);
//        jointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        jointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean elevatorTouchSwitch() {
        return !this.touchDown.getState();
    }

    boolean jointTouchSwitch() {
//        return !this.flipTouchSwitch.getState();
        return false;
    }

    boolean jointTouchSwitch2() {
//        return !this.flipTouchSwitch2.getState();
        return false;
    }

    /**
     * false : CLOSE
     * true : OPEN
     *
     * @param grab grabber state
     */
    public void grabber(boolean grab) {
        grabberLeft.setPosition(grab ? 1 : 0);
        grabberRight.setPosition(grab ? 1 : 0);
    }

    public double getGrabberPosition() {
        return grabberLeft.getPosition();
    }

    public boolean isGrabberClosed() {
        return this.getGrabberPosition() > 0.93;
    }

    public void gotoDescentLevel(Toggle grabberToggle) {
        final int descentGain = 66;
        gotoLevel(LiftLevel.coneStack, -getLiftDescentLevel(1) * descentGain, true, grabberToggle, false);

    }

    public double GetPrevArmPos() {
        return prevArmPos;
    }

    public int getLiftDescentLevel(int increase) { // Increase in descent = decrease in height
        this.liftDescentLevel = (this.liftDescentLevel + increase) % 5;
        return this.liftDescentLevel;
    }


//    public void rotate(boolean rotated) {
//        rotateServo.setPosition(rotated ? 1 : 0);
//    }

    public void toggleFlip(Toggle grabberToggle) {
        if (this.armState != ArmState.Flip) setArmState(ArmState.Flip, grabberToggle);
        else setArmState(ArmState.Home, grabberToggle);
    }

    public enum LiftState {
        Idle, Maintain, // (pos: int),
        Manual, Goto, ControlledGoto;
    }

    public enum ArmPos {
        Left(1), Middle(0.5), Right(0);

        double position;

        ArmPos(double position) {
            this.position = position;
        }
    }

    private LiftState liftState;

    public LiftState getState() {
        return liftState;
    }


    public enum ArmState {
        Home, Flip
    }


    ArmState armState;

    public enum LiftLevel {
        Floor(0), First(1218), Second(2050), Third(2890), ThirdAUTO(2400), ThirdFront(3000), coneStack(475);


        final int position;

        LiftLevel(int p) {
            this.position = p;
        }
    }

    public void update() {
//        if ((this.jointTouchSwitch() |/*| jointMotor.getCurrentPosition() <= 60*/) && armState == ArmState.Home) { // ensures joint doesn't use power when touching touch switch
//            this.jointMotor.setPower(0);
//        }
//        if ((this.jointTouchSwitch2() || /*jointMotor.getCurrentPosition() >= 110*/) && armState == ArmState.Flip) { // ensures joint doesn't use power when touching touch switch
//            this.jointMotor.setPower(0);
//        }
    }

    public void setLiftPower(double pow) {
        if (this.elevatorTouchSwitch() && pow < 0) {
            setLiftState(LiftState.Idle);
            pow = 0;
        }
        if (Math.abs(pow) > liftDeadZone) {
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
//                if (!this.jointMotor.isBusy()) this.setArmState(ArmState.Rotate1);
//                break;
//            case Rotate1:
//
//                break;
//            case Flip:
//                if (!this.jointMotor.isBusy()) this.setArmState(ArmState.Rotate2);
//                break;
//            case Rotate2:
//
//                break;
//            default:
//                break;
//        }
//    }

    public void gotoLevel(LiftLevel level, boolean flip, Toggle grabberToggle, boolean grab) {
        gotoLevel(level, 0, flip, grabberToggle, grab);
    }

    public void gotoLevel(LiftLevel level, boolean flip, Toggle grabberToggle) {
        gotoLevel(level, 0, flip, grabberToggle, false);
    }

    public void gotoLevelSleep(LiftLevel level, int positionDiff, boolean flip, Toggle grabberToggle, int milliseconds, LinearOpMode opMode) {
        this.grabber(true);
        if (grabberToggle != null) grabberToggle.set(true);
        if (grabberLeft.getPosition() == 0 || grabberRight.getPosition() == 0)
            opMode.sleep(milliseconds);
        gotoLevel(level, positionDiff, flip, grabberToggle, false);
    }

    public void gotoLevel(LiftLevel level, int positionDiff, boolean flip, Toggle grabberToggle, boolean grab) {
        gotoLevel(level, positionDiff, flip, grabberToggle, grab, false);
    }


    public void gotoLevel(LiftLevel level, int positionDiff, boolean flip, Toggle grabberToggle, boolean grab, boolean superSpeed) {
        if (grab) {
            this.grabber(true);
            if (grabberToggle != null) grabberToggle.set(true);
        } // TODO: check and fix accordingly (grabber level change -> grabber close).
        if ((level == LiftLevel.Floor || level == LiftLevel.coneStack) && flip) {
            setArmState(ArmState.Flip, grabberToggle);
//            toggleFlip(grabberToggle);
        } else if (flip) setArmState(ArmState.Home, grabberToggle);

        rightElevator.setTargetPosition(level.position + positionDiff);
        leftElevator.setTargetPosition(level.position + positionDiff);
        this.setLiftState(LiftState.Goto);

    }

    public void gotoLevelOnly(LiftLevel level, int positionDiff) {
        rightElevator.setTargetPosition(level.position + positionDiff);
        leftElevator.setTargetPosition(level.position + positionDiff);
        this.setLiftState(LiftState.Goto);
    }

    public void armGotoPosition(double pos, boolean safeLock) {
        if (safeLock && leftElevator.getCurrentPosition() < MIN_HEIGHT_TO_ARM_MOVEMENT) return;
        armServoLeft.setPosition(pos);
        armServoRight.setPosition(pos);
    }

    /**
     * Moves the arm to pos relative to robot
     */
    public void armGotoPosition(ArmPos pos) {
        armGotoPosition(pos.position, true);
    }

    public void armGotoPosition(ArmPos pos, boolean safeLock) {
        armGotoPosition(pos.position, safeLock);
    }

    public double getArmPos() {
        return armServoLeft.getPosition();
    }

    //    public void controlledGotoLevel(LiftLevel level, int positionDiff, double targetPower, int tolerance) {
//        final int deltaPosition = (level.position + positionDiff) - this.leftElevator.getCurrentPosition();
//        final float sign = Math.signum(deltaPosition);
//        final int decelerationRange = 350;
//        final float decelerationGain = (decelerationRange < Math.abs(deltaPosition)) ? sign : (((float)deltaPosition) / ((float)decelerationRange));
//        targetPower = Math.min(targetPower, 1); // Checks power isn't larger than 1
//        final double powerGain = 0.7 * decelerationGain;
//        final int tickDeltaMax = 35;
//        final double deltaPower = (((this.leftElevator.getCurrentPosition() - this.rightElevator.getCurrentPosition()) * powerGain ) / tickDeltaMax);
//
//        this.rightElevator.setPower(targetPower * powerGain + deltaPower);
//        this.leftElevator.setPower(targetPower * powerGain);
//
//
//    }
    public void autoLevelFunction(boolean grab, Toggle grabberToggle, LiftLevel level, boolean flip) {
        //TODO: initiate gotoControlled function
        if (grab) {
            this.grabber(true);
            if (grabberToggle != null) grabberToggle.set(true);
        } // TODO: check and fix accordingly (grabber level change -> grabber close).
        if (level == LiftLevel.Floor || level == LiftLevel.coneStack) {
            setArmState(ArmState.Flip, grabberToggle);
//                  toggleFlip(grabberToggle);
        } else if (flip) setArmState(ArmState.Home, grabberToggle);
        this.leftElevator.setTargetPosition(level.position);
        this.rightElevator.setTargetPosition(level.position);
        this.setLiftState(LiftState.Goto);
        if (camServo.getPosition() == 0) {
            armGotoPosition(ArmPos.Middle, false);
        }
    }

    public void runLift(double liftManualPower, LiftLevel setLiftLevel) {
        final double gotoTargetPower = 1;
        final int gotoTolerance = 70;
        if (setLiftLevel != null) this.currentLevelTarget = setLiftLevel;
        if (Math.abs(liftManualPower) > liftDeadZone) { // manual Power
            this.setLiftPower(liftManualPower * Math.abs(liftManualPower));
            currentLevelTarget = null;
        } else if (currentLevelTarget != null) // go to currentLevelTarget var
            this.controlledGotoLevel(currentLevelTarget, 0, gotoTargetPower, gotoTolerance); // Controlled goto
        if (Math.abs(liftManualPower) < liftDeadZone && currentLevelTarget == null && (!leftElevator.isBusy()) && !(liftState == LiftState.Idle)) { // Stick is 0 and right_elevator isn't busy.
            this.setLiftState(LiftState.Maintain); // Keep current position
        }
    }

    public boolean controlledGotoLevel(LiftLevel level, int positionDiff, double targetPower, int tolerance) {
        final int targetPosition = level.position + positionDiff;
        int deltaPosition = targetPosition - this.leftElevator.getCurrentPosition();
        final double decelerationRange = 100;
        final double minPower = 0.15;
        float sign = Math.signum(deltaPosition);
        deltaPosition = Math.abs(deltaPosition); // turns into absolute value.
        setLiftState(LiftState.ControlledGoto);
        double power_ = 1;
        if (deltaPosition < decelerationRange) power_ -= (deltaPosition / decelerationRange);

        power_ *= (1 - minPower);
        if (sign < 0) power_ *= 0.5;
        if (deltaPosition >= tolerance) {
            setLiftPower((power_ + minPower) * sign * targetPower);
        } else {
            currentLevelTarget = null;
            this.gotoLevelOnly(level, positionDiff);
            return true;
        }
        return false;
    }

    //    public void setLiftState(LiftState newLiftState) {
//        setLiftState(liftState,false);
//    }
    public void setLiftState(LiftState newLiftState) {
        if (newLiftState == this.liftState) return; // State unchanged => do nothing

        switch (newLiftState) {
            case Idle:
                rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightElevator.setPower(0);
                leftElevator.setPower(0);
                // jointMotor.setPower(0);
                break;
            case Manual:
            case ControlledGoto:
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
                //double power = superSpeed ? 0.5 : 0.8;
                rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightElevator.setPower(0.85);
                leftElevator.setPower(0.85);
//                if (leftElevator.getCurrentPosition() >= MIN_HEIGHT_TO_ARM_MOVEMENT && GetCameraServoPos() != 0) {
//                    MoveCameraServo(true);
//                }
                break;
            default:
                break;
        }
        this.liftState = newLiftState;
    }

    public void setArmState(ArmState newState, Toggle grabberToggle) {
        if (newState == this.armState) return;
        switch (newState) {
            case Home:
//                jointMotor.setTargetPosition(0);
                //jointMotor.setTargetPositionTolerance(FLIP_TOLERANCE);
//                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                jointMotor.setPower(1);
//                rotateServo.setPosition(1);
                grabber(true);
                if (grabberToggle != null) grabberToggle.set(true);
                break;
            case Flip:
//                jointMotor.setTargetPosition(FLIP_POSITION);
//                jointMotor.setTargetPositionTolerance(FLIP_TOLERANCE);
//                jointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                jointMotor.setPower(1);
//                rotateServo.setPosition(0);
                grabber(true);
                if (grabberToggle != null) grabberToggle.set(true);
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
//    public static class liftGotoThread extends Thread {
//        public boolean isRunning = true, reachedTarget = true;
//        private final DcMotorEx liftLeft, liftRight;
//        private double targetPower = 0.5;
//        private int tolerance;
//        private int finalTarget = 0;
//
//        public liftGotoThread(DcMotorEx liftRight, DcMotorEx liftLeft, double targetPower, int tolerance, int finalTarget) {
//            this.liftLeft = liftLeft;
//            this.liftRight = liftRight;
//            this.tolerance = tolerance;
//            this.targetPower = targetPower;
//            this.finalTarget = finalTarget;
//        }
//
//        @Override
//        public synchronized void start() {
//            super.start();
//            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        @Override
//        public void run() {
//            super.run();
//            while(isRunning) {
//
//
//                liftMotor.setPower(targetPower);
//
//            }
//
//        }
//    }
//}