package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeBeltMotor {
    private DcMotorEx intakeBeltMotor;

    public IntakeBeltMotor(HardwareMap hardwareMap) {
        intakeBeltMotor = hardwareMap.get(DcMotorEx.class, "intakeBeltMotor");
        intakeBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBeltMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class StartIntakeBeltMotor implements Action {
        @Override
        public synchronized boolean run(@NonNull TelemetryPacket packet) {
//            intakeBeltMotor.setPosition(1.0);
            intakeBeltMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeBeltMotor.setPower(1);

//            try {
//                wait(3000);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }

            return false;
        }
    }
    public Action startIntakeBeltMotor() {
        return new StartIntakeBeltMotor();
    }

    public class StopIntakeBeltMotor implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            intakeBeltMotor.setPosition(0.5);
            intakeBeltMotor.setPower(0);
            return false;
        }
    }
    public Action stopIntakeBeltMotor() {
        return new StopIntakeBeltMotor();
    }

    public class ReverseIntakeBeltMotor implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            intakeBeltMotor.setPosition(0);
            intakeBeltMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeBeltMotor.setPower(0.4);
            return false;
        }
    }
    public Action reverseIntakeBeltMotor() {
        return new ReverseIntakeBeltMotor();
    }

    public DcMotorEx getIntakeBeltMotor() {
        return intakeBeltMotor;
    }
}
