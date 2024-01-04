package org.firstinspires.ftc.teamcode.centerstage.Systems.Arm;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.GoToState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.GraceHoldTimeState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.HoldState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.IdleState;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.States.ManualState;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.State;
import org.firstinspires.ftc.teamcode.centerstage.util.StateMachine.StateMachine;

public class Arm implements Runnable {
    DcMotorEx lift1 = null, lift2 = null;
    Servo leftClawServo = null, rightClawServo = null;
    private final LinearOpMode opMode;
    private final double deadZone = 0.05;
    private final double holdPower = 0.2;
    int targetPos;
    public ElapsedTime timer;
    public double graceTimeLimit = 0.25;
    public State<Arm> gotoState, graceHoldTimeState, holdState, idleState, manualState;
    Telemetry telemetry;

    public double getHoldPower() {
        return holdPower;
    }


    public enum Position {
        One(500), Two(900), Three(1200); // Todo : put actual values
        final int liftPos;

        Position(int liftPos) {
            this.liftPos = liftPos;
        }
    }

    public StateMachine<Arm> stateMachine;

    public Arm(LinearOpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.opMode = opMode;
        init(opMode.hardwareMap);
    }

    public void init(HardwareMap hw) {
        //region States Init
        gotoState = new GoToState();
        graceHoldTimeState = new GraceHoldTimeState();
        holdState = new HoldState();
        idleState = new IdleState();
        manualState = new ManualState();
        stateMachine = new StateMachine<>(this);
        //endregion
        timer = new ElapsedTime();
        //region Motors Init
        telemetry.addData("Lift Init", "In progress");
        telemetry.update();
        lift1 = hw.get(DcMotorEx.class, "lift1");
        lift2 = hw.get(DcMotorEx.class, "lift2");
        lift1.setDirection(DcMotorEx.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion
        //region Claw Init
        leftClawServo = hw.get(Servo.class, "leftClawServo");
        leftClawServo = hw.get(Servo.class, "rightClawServo");
        setClawPosition(false);
        //endregion
        telemetry.addData("Lift Init", "Finished");
        telemetry.update();
        resetArm();
        stateMachine.changeState(idleState);
    }

    public void resetArm() {
        // TODO: Make initialization sequence
        leftClawServo.setPosition(0);
        leftClawServo.setPosition(0);
    }

    public void run() {
        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            handleStates();
            opMode.idle();
        }
    }

    public boolean isOverCurrentLimit() {
        return getCurrent() > 1.5;
    }

    public void goToPos(Position pos) {
        goToPos(pos.liftPos);
    }

    public void goToPos(int pos) {
        setTargetPositions(pos);
        stateMachine.changeState(gotoState);
    }

    public int getPos() {
        return lift1.getCurrentPosition();
    }

    public int getTargetPos() {
        return this.targetPos;
    }

    public void setTargetPositions(int pos) {
        targetPos = pos;
    }

    public void setManualMode(boolean manual, double power) {
        if (manual) {
            setMotorsPower(0);
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

    public void handleStates() {
        stateMachine.execute();
    }

    public boolean areMotorsBusy() {
        return !lift1.isBusy() && !lift2.isBusy();
    }


    public void setPower(double power) {
        if (outOfDeadZone(power)) {
            stateMachine.changeState(manualState);
            setMotorsPower(power);
        } else {
            if (stateMachine.getCurrentState() != gotoState && stateMachine.getCurrentState() != holdState) {
                setTargetPositions(getPos());
                stateMachine.changeState(holdState);
            }
        }
    }

    public void setMotorsPower(double power) {
        lift1.setPower(power);
        lift2.setPower(power);
    }

    public double getCurrent() {
        return lift1.getCurrent(CurrentUnit.AMPS);
    }

    public boolean outOfDeadZone(double power) {
        return Math.abs(power) > deadZone;
    }

    public void setClawPosition(double position) {
        leftClawServo.setPosition(position);
        leftClawServo.setPosition(position);
    }

    public void setClawPosition(boolean open) {
        setClawPosition(open ? 1 : 0);
    }
}
