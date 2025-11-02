package org.firstinspires.ftc.teamcode.decode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMotor {

    private DcMotorEx intakeMotor;

    public IntakeMotor(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class StartIntake implements Action{


        @Override
        public synchronized boolean run(@NonNull TelemetryPacket packet){
            intakeMotor.setPower(0.75);

//            try {
//                wait(0);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }

            return false;
        }
    }
    public Action startIntake(){return new StartIntake();}

    public class StopIntake implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            intakeMotor.setPower(0);

            return false;
        }
    }
    public  Action stopIntake(){return new StopIntake();}

//    public DcMotorEx getIntakeMotor(){return intakeMotor;}


}
