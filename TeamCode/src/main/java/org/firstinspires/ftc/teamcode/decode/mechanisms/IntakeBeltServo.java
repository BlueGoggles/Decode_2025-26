package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.helper.Constants;

public class IntakeBeltServo {
    private Servo intakeBeltServo;

    public IntakeBeltServo(HardwareMap hardwareMap) {
        intakeBeltServo = hardwareMap.get(Servo.class, "intakeBeltServo");
        intakeBeltServo.setDirection(Servo.Direction.REVERSE);
        intakeBeltServo.setPosition(Constants.SERVO_HOME_POSITION);
    }

    public class StartIntakeBeltServo implements Action {
        @Override
        public synchronized boolean run(@NonNull TelemetryPacket packet) {
            intakeBeltServo.setPosition(1.0);

//            try {
//                wait(3000);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }

            return false;
        }
    }
    public Action startIntakeBeltServo() {
        return new StartIntakeBeltServo();
    }

    public class StopIntakeBeltServo implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeBeltServo.setPosition(0.5);
            return false;
        }
    }
    public Action stopIntakeBeltServo() {
        return new StopIntakeBeltServo();
    }

    public class ReverseIntakeBeltServo implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeBeltServo.setPosition(0);
            return false;
        }
    }
    public Action reverseIntakeBeltServo() {
        return new ReverseIntakeBeltServo();
    }

    public Servo getIntakeBeltServo() {
        return intakeBeltServo;
    }
}
