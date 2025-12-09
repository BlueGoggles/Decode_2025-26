package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.helper.Constants;

public class KickerStopperServo {

    private Servo kickerStopperServo;

    public KickerStopperServo(HardwareMap hardwareMap) {
//        kickerStopperServo = hardwareMap.get(Servo.class, "kickerStopperServo");
//        kickerStopperServo.setDirection(Servo.Direction.REVERSE);
//        kickerStopperServo.setPosition(0);
    }

    public class KickerStopperServoOn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            kickerStopperServo.setPosition(0);
            return false;
        }
    }

    public Action kickerStopperServoOn() {

        return new KickerStopperServoOn();
    }

    public class KickerStopperServoOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            kickerStopperServo.setPosition(0.3);
            return false;
        }
    }

    public Action kickerStopperServoOff() {

        return new KickerStopperServoOff();
    }
}
