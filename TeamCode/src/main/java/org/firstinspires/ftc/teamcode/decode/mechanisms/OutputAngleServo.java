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
        outputAngleServo.setPosition(Constants.DEFAULT_OUTPUT_ANGLE_POSITION_2);
    }

    public class OutputAngleAction implements Action {

        String launchLocation;
        public OutputAngleAction(String launchLocation) {
            this.launchLocation = launchLocation;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            if (launchLocation.equals(Constants.BLUE_LAUNCH_LOCATION_1)) {
                outputAngleServo.setPosition(Constants.DEFAULT_OUTPUT_ANGLE_POSITION_1);
            } else if (launchLocation.equals(Constants.BLUE_LAUNCH_LOCATION_2)) {
                outputAngleServo.setPosition(Constants.DEFAULT_OUTPUT_ANGLE_POSITION_2);
            } else if(launchLocation.equals(Constants.BLUE_LAUNCH_LOCATION_3)) {
                outputAngleServo.setPosition(Constants.DEFAULT_OUTPUT_ANGLE_POSITION_3);
            } else if(launchLocation.equals(Constants.RED_LAUNCH_LOCATION_2_AUTON)){
               outputAngleServo.setPosition(Constants.DEFAULT_OUTPUT_ANGLE_POSITION_2_AUTON);
            } else if(launchLocation.equals(Constants.RED_LAUNCH_LOCATION_3_AUTON)){
                outputAngleServo.setPosition(Constants.DEFAULT_OUTPUT_ANGLE_POSITION_3_AUTON);
            } else if(launchLocation.equals(Constants.RED_LAUNCH_LOCATION_1_AUTON)){
                outputAngleServo.setPosition(Constants.DEFAULT_OUTPUT_ANGLE_POSITION_1_AUTON);
            }
            return false;
        }

    }

    public Action setOutputAngle(String launchLocation){
        return new OutputAngleAction(launchLocation);
    }

    public class AdjustOutputAngle implements Action {

        boolean increaseFlag;

        public AdjustOutputAngle(boolean increaseFlag) {
            this.increaseFlag = increaseFlag;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(this.increaseFlag) {
                outputAngleServo.setPosition(outputAngleServo.getPosition() + 0.005);
            } else {
                outputAngleServo.setPosition(outputAngleServo.getPosition() - 0.005);
            }
            return false;
        }
    }

    public Action adjustOutputAngle(boolean increaseFlag) {
        return new AdjustOutputAngle(increaseFlag);
    }

    public Servo getOutputAngleServo() {
        return outputAngleServo;
    }
}
