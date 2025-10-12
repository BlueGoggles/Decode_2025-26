package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.decode.helper.Constants;

public class KickerServo {
    private Servo kickerServo;

    public KickerServo(HardwareMap hardwareMap) {
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        kickerServo.setDirection(Servo.Direction.REVERSE);
        kickerServo.setPosition(Constants.SERVO_HOME_POSITION);
    }

    public class StartKickerServo implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            kickerServo.setPosition(1.0);
            return false;
        }
    }
    public Action startKickerServo() {

        return new StartKickerServo();
    }

    public class StopKickerServo implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            kickerServo.setPosition(0.5);
            return false;
        }
    }
    public Action stopKickerServo() {
        return new StopKickerServo();
    }
}
