package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeMotor;
import org.firstinspires.ftc.teamcode.decode.mechanisms.IntakeBeltServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.KickerServo;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.decode.mechanisms.Shooter2;

@TeleOp(name = "Main Teleop", group = "robot")
public class MainTeleOP extends LinearOpMode {


    @Override
    public void runOpMode(){

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeBeltServo intakeBeltServo = new IntakeBeltServo(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Shooter2 shooter2 = new Shooter2(hardwareMap);
        KickerServo kickerServo = new KickerServo(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.a){
                Actions.runBlocking(
                        new ParallelAction(
                                intakeMotor.startIntake(),
                                intakeBeltServo.startIntakeBeltServo()
                        )
                );
            }

            if(gamepad1.b){
                Actions.runBlocking(
                        new ParallelAction(
                                intakeMotor.stopIntake(),
                                intakeBeltServo.stopIntakeBeltServo()

                        )
                );
            }

            if(gamepad1.x){
                Actions.runBlocking(
                        new ParallelAction(
                                shooter.startShooter()
                        )

                );
            }

            if(gamepad1.y){
                Actions.runBlocking(
                        new ParallelAction(
                                shooter.stopShooter()
                        )
                );
            }

            if(gamepad1.start){
                Actions.runBlocking(
                        new ParallelAction(
                                shooter2.startShooter2(),
                                kickerServo.startKickerServo()
                        )

                );
            }

            if(gamepad1.back){
                Actions.runBlocking(
                        new ParallelAction(
                                shooter2.stopShooter2(),
                                kickerServo.stopKickerServo()
                        )
                );
            }

        }



    }
}
