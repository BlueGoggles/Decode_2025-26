package org.firstinspires.ftc.teamcode.decode.helper;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.OutputAngleServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.opmodes.FrontBlue;

public class Utility {

    public static void shoot(LinearOpMode opMode, OutputAngleServo outputAngleServo, Shooter shooter, IntakeMotor intakeMotor, IntakeBeltServo intakeBeltServo, KickerServo kickerServo) throws InterruptedException {
        Actions.runBlocking(
                new SequentialAction(
                        outputAngleServo.setOutputAngle(Constants.BLUE_LAUNCH_LOCATION_2),
                        shooter.startShooter()
                )
        );
//        wait(500);
//        Actions.runBlocking(
//                new SequentialAction(
//                        kickerStopperServo.kickerStopperServoOff()
//                )
//        );
        opMode.wait(500);
        Actions.runBlocking(
                new ParallelAction(
                        intakeMotor.startIntake(),
                        intakeBeltServo.startIntakeBeltServo(),
                        kickerServo.startKickerServo(kickerServo.getKickerServo().getPosition() + 0.13)
                )
        );
        opMode.wait(5000);
        Actions.runBlocking(
                new SequentialAction(
                        kickerServo.startKickerServo(kickerServo.getKickerServo().getPosition() + 0.1)
                )
        );
        opMode.wait(2000);

        Actions.runBlocking(
                new SequentialAction(
                        kickerServo.startKickerServo(Constants.KICKER_SERVO_HOME_POSITION)
                )
        );
    }
}
