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
//        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
//        kickerServo.setDirection(Servo.Direction.FORWARD);
//        kickerServo.setPosition(Constants.KICKER_SERVO_HOME_POSITION);
    }

    public class StartKickerServo implements Action {

        double position;

        public StartKickerServo(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            kickerServo.setPosition(this.position);
            return false;
        }
    }
    public Action startKickerServo(double position) {

        return new StartKickerServo(position);
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

    public Servo getKickerServo() {
        return kickerServo;
    }
}
