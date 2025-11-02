package org.firstinspires.ftc.teamcode.decode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class TrajectoryActions {

    TrajectoryActionBuilder builder;

    public TrajectoryActions(TrajectoryActionBuilder builder) {
        this.builder = builder;
    }



     public Action getTrajectory_1_1(){

         this.builder = this.builder.endTrajectory().fresh()
                 .strafeToSplineHeading(new Vector2d(-5, 62), Math.toRadians(-45));

         return this.builder.build();
     }

    public Action getTrajectory_1_2(){

        this.builder = this.builder.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-10, 62), 135);

        return builder.build();
    }

    public Action getTrajectory_1_3(){

        this.builder = this.builder.endTrajectory().fresh()
                 .lineToYSplineHeading(33, Math.toRadians(135));

        return builder.build();
    }
}
