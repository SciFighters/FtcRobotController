package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSystem {
    Servo intakeServo1, intakeServo2;
    DcMotorEx intakeWheelsMotor;
    LinearOpMode opMode;
    HardwareMap hw;

    public enum GatherSystemServoPos {
        Open(1),
        Close(0);
        private final double servoPos;

        GatherSystemServoPos(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    public enum IntakeWheelsState {
        Collect,
        Spit,
        Idle
    }

    private boolean isBusy;
    private IntakeWheelsState intakeWheelsState;

    public IntakeSystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    public boolean IsBusy() {
        return isBusy;
    }


    public void setStateIdle() {
        setState(IntakeWheelsState.Idle);

    }

    public void setStateSpit() {
        setState(IntakeWheelsState.Spit);
    }

    public void setStateCollect() {
        setState(IntakeWheelsState.Collect);

    }

    public void init(HardwareMap hw) {
        this.hw = hw; // Cache the hardware map
        intakeWheelsMotor = hw.get(DcMotorEx.class, "intakeWheelsMotor");
        setState(IntakeWheelsState.Idle);

        intakeServo1 = hw.get(Servo.class, "intakeServo1");
        intakeServo2 = hw.get(Servo.class, "intakeServo2");
        intakeServo2.setDirection(Servo.Direction.REVERSE);
    }

    void setServoPos(double pos) {
        intakeServo1.setPosition(pos);
        intakeServo2.setPosition(pos);
    }

    void setState(IntakeWheelsState state) {
        this.intakeWheelsState = state;
    }

    /**
     * Moves the intake wheels
     */
    public void spinMotor() {
        double normalPower = 0.4;
        double spitPower = 0.3;
        if (isBusy)
            return; // Don't move if the system is busy
        if (intakeWheelsState == IntakeWheelsState.Spit) {
            setServoPos(GatherSystemServoPos.Close.servoPos);
        } else if (intakeWheelsState == IntakeWheelsState.Idle) {
            setServoPos(GatherSystemServoPos.Close.servoPos);
            intakeWheelsMotor.setPower(0);
            return;
        } else {
            setServoPos(GatherSystemServoPos.Open.servoPos);
        }
        intakeWheelsMotor.setDirection( // Switch direction according to what state its in
                intakeWheelsState == IntakeWheelsState.Spit ?
                        DcMotorEx.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        intakeWheelsMotor.setPower(intakeWheelsState == IntakeWheelsState.Spit ? spitPower : normalPower);
    }

    public IntakeWheelsState getIntakeWheelsState() {
        return this.intakeWheelsState;
    }
}
