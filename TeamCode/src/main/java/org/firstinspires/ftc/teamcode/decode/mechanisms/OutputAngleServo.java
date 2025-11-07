package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.decode.helper.Constants;

public class OutputAngleServo {
    public Servo outputAngleServo;

    public OutputAngleServo(HardwareMap hardwareMap){
        outputAngleServo = hardwareMap.get(Servo.class, "outputAngleServo");
        outputAngleServo.setDirection(Servo.Direction.FORWARD);
        outputAngleServo.setPosition(0.45);
    }

    public class OutputAngleAction implements Action {

        String launchLocation;
        public OutputAngleAction(String launchLocation) {
            this.launchLocation = launchLocation;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            if (launchLocation.equals(Constants.BLUE_LAUNCH_LOCATION_1)) {
                outputAngleServo.setPosition(0.12);
            } else if (launchLocation.equals(Constants.BLUE_LAUNCH_LOCATION_2)) {
                outputAngleServo.setPosition(0.45);
            } else if(launchLocation.equals(Constants.BLUE_LAUNCH_LOCATION_3)) {
                outputAngleServo.setPosition(0.14);
            } else if(launchLocation.equals(Constants.RED_LAUNCH_LOCATION_1)){
               outputAngleServo.setPosition(0);
            } else if(launchLocation.equals(Constants.RED_LAUNCH_LOCATION_2)){
                outputAngleServo.setPosition(0);
            } else if(launchLocation.equals(Constants.RED_LAUNCH_LOCATION_3)){
                outputAngleServo.setPosition(0);
            }
            return false;
        }

    }

    public Action setOutputAngle(String launchLocation){
        return new OutputAngleAction(launchLocation);
    }
}
