package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.*;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.Systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.ThreadedComponent;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;

/**
 * The Arm class controls the movement of an arm using two DC motors.
 */
@ThreadedComponent
public class Arm extends Component {
    public static Arm instance;
    public static double holdPower = 0.1;
    private final double deadZone = 0.05;
    private final double graceTimeLimit = 0.25;
    public State<Arm> gotoState, graceHoldTimeState, holdState, idleState, manualState;
    public DcMotorEx lift1, lift2;
    public TouchSensor limitSensor1;
    public StateMachine<Arm> stateMachine;
    public boolean isPlacePixelBusy = false;
    DriveClass drive;
    private int targetPos;
    private ElapsedTime timer;
    // Constants for proportional control in GoToState
    private Servo frontServo, backServo;
    private DistanceSensor distanceSensor;
    private IntakeSystem intakeSystem;

    public Arm() {

    }

    /**
     * Initializes the Arm object.
     */
    @Override
    public void init() {
        initStateMachine();
        this.timer = new ElapsedTime();
        this.lift1 = hw.get(DcMotorEx.class, "lift1");
        this.limitSensor1 = hw.get(TouchSensor.class, "armLimitSensor1");
        this.lift2 = hw.get(DcMotorEx.class, "lift2");
        this.lift1.setDirection(DcMotorEx.Direction.FORWARD);
        this.lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontServo = hw.get(Servo.class, "frontClawServo");
        this.backServo = hw.get(Servo.class, "backClawServo");
        this.distanceSensor = hw.get(DistanceSensor.class, "armDistanceSensor");
        initMotors();
        resetClaw();
        resetArm();
        closeClaw(true);
        stateMachine.changeState(idleState);
        instance = this;
    }

    private void initStateMachine() {
        gotoState = new GoToState();
        graceHoldTimeState = new GraceHoldTimeState();
        holdState = new HoldState();
        idleState = new IdleState();
        manualState = new ManualState();
        stateMachine = new StateMachine<>(this);
    }

    @Override
    public void start() {
        intakeSystem = robot.getComponent(IntakeSystem.class);
        drive = robot.getComponent(DriveClass.class);
    }

    private void initMotors() {
        telemetry.addData("Lift Init", "In progress");
        telemetry.update();

        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Lift Init", "Finished");
        telemetry.update();
    }

    public void resetClaw() {
        double minServoPos = 0.35;
        frontServo.scaleRange(minServoPos, 1);
        backServo.scaleRange(minServoPos, 1);
    }

    /**
     * Resets the arm to its initial state. To be implemented.
     */
    public void resetArm() {
        setTargetPositions(0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!limitSensor1.isPressed()) {
            while (!limitSensor1.isPressed() && (!robot.isStopRequested()) && timer.seconds() < 1.5) {
                lift1.setPower(-0.3);
                lift2.setPower(-0.3);
                telemetry.addData("BUTTON STATUS: ", limitSensor1.isPressed());
                telemetry.update();
            }
        }
        lift1.setPower(0);
        lift2.setPower(0);

        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        setTargetPositions(0);
    }

    /**
     * Runs the Arm thread continuously, handling state transitions.
     */
    @Override
    public void update() {
        handleStates();
//        if (pos() < 800 && pos() > 60 && intakeSystem.state() != IntakeSystem.WheelsState.Collect && !intakeSystem.initTime) {
//            intakeSystem.setStateAvoidArm();
//        }
        if ((Math.abs(robot.gamepad2.left_stick_x) > 0.05 || Math.abs(robot.gamepad2.left_stick_y) > 0.05) && holdPower != 0.1) {
            holdPower = 0.1;
        }
    }

    /**
     * Checks if the current drawn by the motors exceeds a limit.
     *
     * @return True if the current is over the limit, false otherwise.
     */
    public boolean isOverCurrentLimit() {
        return getCurrent() > 1.5; // Adjust this limit based on your requirements
    }

    /**
     * Moves the arm to the specified position using fixed control.
     *
     * @param pos The target position for the arm.
     */
    public void goToPos(Position pos) {
        goToPos(pos.liftPos);
    }

    /**
     * Moves the arm to the specified position using proportional control.
     *
     * @param pos The target position for the arm.
     */
    public void goToPos(int pos) {
        setTargetPositions(pos);
        stateMachine.changeState(gotoState);
    }

    /**
     * @param position arm height
     */
    public void placePixels(Position position) {
        isPlacePixelBusy = true;
        double distance = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());
        if (distance < position.distanceFromBackboard && pos() > position.liftPos) {
            alignToBoard(position);
            goToPos(position);
        } else {
            goToPos(position);
            alignToBoard(position);
        }
        isPlacePixelBusy = false;
    }

    void alignToBoard(Position position) {
        double distance = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());

        while (!MathUtil.approximately(distance, position.distanceFromBackboard, 1)) {
            double delta = position.distanceFromBackboard - distance;
            double gain = 0.016;
            double power = gain * delta;

            robot.telemetry.addData("Place pixel delta distance", delta);
            robot.telemetry.addData("power", power);
            robot.telemetry.update();

            drive.setPowerOriented(0, power, 0, true);
            distance = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());
            if (Math.abs(robot.gamepad1.left_stick_x) > 0.1 || Math.abs(robot.gamepad1.left_stick_y) > 0.1 || Math.abs(robot.gamepad2.left_stick_x) > 0.1 || Math.abs(robot.gamepad2.left_stick_y) > 0.1) {
                break;
            }
        }
    }

    /**
     * Gets the current position of the arm.
     *
     * @return The current position in encoder counts.
     */
    public int pos() {
        return lift1.getCurrentPosition();
    }

    /**
     * Gets the target position for the arm.
     *
     * @return The target position in encoder counts.
     */
    public int targetPos() {
        return this.targetPos;
    }

    /**
     * Sets the target positions for the arm.
     *
     * @param pos The target position in encoder counts.
     */
    public void setTargetPositions(int pos) {
        targetPos = pos;
    }

    /**
     * Sets the manual control mode and power for the arm.
     *
     * @param manual True to enable manual mode, false for position control.
     * @param power  The manual power if in manual mode.
     */
    public void setManualMode(boolean manual, double power) {
        if (manual) {
            lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            lift1.setTargetPosition(targetPos);
            lift2.setTargetPosition(targetPos);

            lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            setMotorsPower(power);
        }
    }

    /**
     * Handles the state machine execution.
     */
    public void handleStates() {
        stateMachine.execute();
        if (limitSensor1.isPressed()) {
            resetArm();
        }
    }

    /**
     * Checks if the motors are currently busy with a task.
     *
     * @return True if the motors are busy, false otherwise.
     */
    public boolean areMotorsBusy() {
        return !lift1.isBusy() && !lift2.isBusy();
    }

    /**
     * Sets the power for the arm motors.
     *
     * @param power The power to set to the motors.
     */
    public void setPower(double power) {
        if (outOfDeadZone(power)) {
            stateMachine.changeState(manualState);
            setMotorsPower(power);
        } else {
            if (stateMachine.getCurrentState() != gotoState) {
                setTargetPositions(pos());
                stateMachine.changeState(holdState);
            }
        }
    }

    public double distanceSensorDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Sets the power for the arm motors.
     *
     * @param power The power to set to the motors.
     */
    public void setMotorsPower(double power) {
        final int higherLimit = 40, lowerLimit = 5;
        if (Math.abs(power) < 0.05) {
            power = 0;
        } else if ((power < 0) && (limitSensor1.isPressed()) || (power > 0 && distanceSensorDistance() < lowerLimit && pos() > 1000)) {
            return;
        } else if (pos() < 800 && outOfDeadZone(power)) {
//            intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
        } else if (distanceSensorDistance() < higherLimit && pos() > 1000 && power > 0) {
            if (stateMachine.getCurrentState() == gotoState && targetPos() < pos()) {
            } else {
                power = Math.abs(MathUtil.map(power, lowerLimit, higherLimit, 0, 1));
            }
        } // else if (pos() < 1000 && pos() > 60 && intakeSystem.state != IntakeSystem.WheelsState.Collect && !intakeSystem.initTime) {
//            intakeSystem.state = IntakeSystem.WheelsState.AvoidArm;
//            intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
//            intakeSystem.spinMotor();
//        }
//        if (outOfDeadZone(power) && power > 0 && intakeSystem.state == IntakeSystem.WheelsState.Collect) {
//            setClawPosition(true);
//            intakeSystem.setStateIdle();
//        }
        else if (targetPos() == 0) {
//            resetArm();
        }
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public void setManualPower(double power) {
        stateMachine.changeState(manualState);
        setMotorsPower(power);
    }

    /**
     * Gets the current drawn by the arm motors.
     *
     * @return The current drawn by the motors in amps.
     */
    public double getCurrent() {
        return lift1.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Checks if the power is outside the dead zone.
     *
     * @param power The power to be checked.
     * @return True if the power is outside the dead zone, false otherwise.
     */
    public boolean outOfDeadZone(double power) {
        return Math.abs(power) > deadZone;
    }

    /**
     * Sets the position of the claw servos.
     *
     * @param position The position to set the claw servos to.
     */
    public void closeClaw(double position) {
        frontServo.setPosition(position);
        backServo.setPosition(position);
    }

    /**
     * Sets the position of the claw servos based on open or closed state.
     *
     * @param close True to close the claw, false to open it.
     */
    public void closeClaw(boolean close) {
        closeClaw(close ? 1 : 0);
    }

    public double clawPosition() {
        return backServo.getPosition();
    }

    public double getDeadZone() {
        return this.deadZone;
    }

    public void goHome() {
        closeClaw(false);
        goToPos(0);
    }

    public void dropAndReturn() {
        closeClaw(false);
        robot.sleep(300);
        goToPos(0);
    }

    public void hang() {
        holdPower = 0.15;
        goToPos(Position.Hang);
    }

    /**
     * Enumeration for different arm positions.
     */
    public enum Position {
        Home(0, -1), One(2700, 40), Two(2525, 27), Hang(500, -1), Three(2245, 7);
        // Todo: put actual values
        public final int liftPos;
        public final double distanceFromBackboard;

        Position(int liftPos, double distanceFromBackboard) {
            this.liftPos = liftPos;
            this.distanceFromBackboard = distanceFromBackboard;
        }
    }
}
