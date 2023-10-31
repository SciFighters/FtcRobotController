package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GatherSystem {
    DcMotorEx intakeWheelsMotor;
    LinearOpMode opMode;
    HardwareMap hw;

//    public enum GatherSystemState {
//        Open(0),
//        Close(1);
//        private final double servoPos;
//
//        GatherSystemState(double servoPos) {
//            this.servoPos = servoPos;
//        }
//    }

    public enum IntakeWheelsState {
        Collect,
        Spit,
        Idle
    }

    private boolean isBusy;
    private IntakeWheelsState intakeWheelsState;

    public GatherSystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    public boolean IsBusy() {
        return isBusy;
    }

    public void setGatherSystemState(IntakeWheelsState state) {
        this.intakeWheelsState = state;
    }

    public void setStateIdle() {
        this.intakeWheelsState = IntakeWheelsState.Idle;
    }

    public void setStateSpit() {
        this.intakeWheelsState = IntakeWheelsState.Spit;
    }

    public void setStateCollect() {
        this.intakeWheelsState = IntakeWheelsState.Collect;
    }

    public void init(HardwareMap hw) {
        this.hw = hw; // Cache the hardware map
        intakeWheelsMotor = hw.get(DcMotorEx.class, "intakeWheelsMotor");
        setGatherSystemState(IntakeWheelsState.Idle);
    }

    /**
     * Moves the intake wheels
     *
     * @param power Should always be positive, direction is handled in the class
     */
    public void spinMotor(double power) {
        if (isBusy || intakeWheelsState == IntakeWheelsState.Idle)
            return; // Don't move if the system is busy
        intakeWheelsMotor.setDirection( // Switch direction according to what state its in
                intakeWheelsState == IntakeWheelsState.Spit ?
                        DcMotorEx.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        intakeWheelsMotor.setPower(power);
    }
}
