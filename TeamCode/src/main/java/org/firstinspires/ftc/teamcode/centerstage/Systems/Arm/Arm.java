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
        One(-500), Two(-900), Three(-1200); // Todo : put actual values
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
        gotoState = new GoToState();
        graceHoldTimeState = new GraceHoldTimeState();
        holdState = new HoldState();
        idleState = new IdleState();
        manualState = new ManualState();
        timer = new ElapsedTime();
        stateMachine = new StateMachine<>(this);
        telemetry.addLine("Lift Init");
        telemetry.update();
        lift1 = hw.get(DcMotorEx.class, "lift1");
        lift2 = hw.get(DcMotorEx.class, "lift2");
        lift1.setDirection(DcMotorEx.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stateMachine.changeState(idleState);

//        while (opMode.hardwareMap.voltageSensor.get("Control Hub").getVoltage() > 12) {
//            setMotorsPower(0.3);
////            opMode.sleep(200);
//            opMode.telemetry.update();
//        }
//        setMotorsPower(0);


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
        setTargetPositions(pos.liftPos);
        stateMachine.changeState(gotoState);
    }

    public int getPos() {
        return lift1.getCurrentPosition();
    }

    public void setTargetPositions(int pos) {
        targetPos = pos;
    }

    public void setManualMode(boolean manual, double power) {
        if (manual) {
            lift1.setPower(0);
            lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            lift2.setPower(0);
            lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            lift1.setTargetPosition(targetPos);
            lift1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift1.setPower(power);

            lift2.setTargetPosition(targetPos);
            lift2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lift2.setPower(power);
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
            if (stateMachine.getCurrentState() != gotoState)
                stateMachine.changeState(holdState);
        }

        handleStates();
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

}
