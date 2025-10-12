package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.helper.Constants;

public class IntakeBeltServo {
    private Servo intakeBeltServo;

    public IntakeBeltServo(HardwareMap hardwareMap) {
        intakeBeltServo = hardwareMap.get(Servo.class, "intakeBeltServo");
        intakeBeltServo.setDirection(Servo.Direction.FORWARD);
        intakeBeltServo.setPosition(Constants.SERVO_HOME_POSITION);
    }

    public class StartIntakeBeltServo implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeBeltServo.setPosition(1.0);
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

//    public Servo getIntakeBeltServoServo() {
//        return intakeBeltServo;
//    }
}
