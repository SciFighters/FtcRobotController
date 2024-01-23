package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.*;
import org.firstinspires.ftc.teamcode.centerstage.Systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.ThreadedComponent;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.StateMachine;

/**
 * The Arm class controls the movement of an arm using two DC motors.
 */
@ThreadedComponent
public class Arm extends Component {
    private static Arm instance;
    private final double deadZone = 0.05;
    private double manualPower = 0;
    private int targetPos;
    private ElapsedTime timer;
    private final double graceTimeLimit = 0.25;
    public State<Arm> gotoState, graceHoldTimeState, holdState, idleState, manualState;
    public DcMotorEx lift1, lift2;
    public TouchSensor limitSensor1;
    // Constants for proportional control in GoToState
    private static final double PROPORTIONAL_GAIN = 0.01; // Adjust this gain based on your requirements
    private Servo frontServo, backServo;
    public StateMachine<Arm> stateMachine;
    private DistanceSensor distanceSensor;

    public Arm() {

    }

    /**
     * Enumeration for different arm positions.
     */
    public enum Position {
        One(500), Two(900), Three(2120); // Todo: put actual values
        final int liftPos;

        Position(int liftPos) {
            this.liftPos = liftPos;
        }
    }

    public static Arm getInstance() {
        return instance;
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
        setClawPosition(true);
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
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!limitSensor1.isPressed()) {
            robot.getComponent(IntakeSystem.class).setServoPos(IntakeSystem.ServoPos.Mid);
            while (!limitSensor1.isPressed() && (robot.opModeInInit() && !robot.isStopRequested())) {
                lift1.setPower(-0.2);
                lift2.setPower(-0.2);
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
    public void loop() {
        handleStates();
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
     * Moves the arm to the specified position using proportional control.
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
     * Gets the current position of the arm.
     *
     * @return The current position in encoder counts.
     */
    public int getPos() {
        return lift1.getCurrentPosition();
    }

    /**
     * Gets the target position for the arm.
     *
     * @return The target position in encoder counts.
     */
    public int getTargetPos() {
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
            lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift2.setTargetPosition(targetPos);
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
            manualPower = power;
        } else {
            manualPower = 0;
            if (stateMachine.getCurrentState() != gotoState && stateMachine.getCurrentState() != holdState) {
                setTargetPositions(getPos());
                stateMachine.changeState(holdState);
            }
        }
    }

    /**
     * Gets the current manual power setting.
     *
     * @return The current manual power setting.
     */
    public double getManualPower() {
        return manualPower;
    }


    public double getDistanceSensorDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Sets the power for the arm motors.
     *
     * @param power The power to set to the motors.
     */
    public void setMotorsPower(double power) {
        if ((power <= 0) && (limitSensor1.isPressed())) {
            return;
        } else if (power > 0 && getDistanceSensorDistance() < 10 && getPos() > 1000) {
            return;
        }
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public void setManualPower(double power) {
        if (outOfDeadZone(power)) {
            stateMachine.changeState(manualState);
            setMotorsPower(power);
        }
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
    public void setClawPosition(double position) {
        frontServo.setPosition(position);
        backServo.setPosition(position);
    }

    /**
     * Sets the position of the claw servos based on open or closed state.
     *
     * @param open True to open the claw, false to close it.
     */
    public void setClawPosition(boolean open) {
        setClawPosition(open ? 1 : 0);
    }

    public double getClawPosition() {
        return backServo.getPosition();
    }

    public double getDeadZone() {
        return this.deadZone;
    }

    public void goHome() {
        robot.getComponent(IntakeSystem.class).setStateCollect();
        goToPos(0);
        robot.getComponent(IntakeSystem.class).setStateIdle();
        setClawPosition(true);
    }
}
