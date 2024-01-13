package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
//TODO                       Red Right    Left
                .lineToY(-37)
                .turn(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(8, -35), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(15,-35), Math.toRadians(-90))
                        .turn(Math.toRadians(-45))
                        .lineToY(-58)
                .turn(Math.toRadians(-90))
                .lineToX(47)
                .strafeToConstantHeading(new Vector2d(47,-29))
                        .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(12,-60),Math.toRadians(90))
                                .turn(Math.toRadians(90))
//TODO                      Red Right    Middle
                                .lineToY(-35)
                                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .lineToX(47)
                .strafeToConstantHeading(new Vector2d(47,-34))
                        .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(12,-60),Math.toRadians(90))
                .turn(Math.toRadians(90))
//TODO                            Red Right    Right
                .splineToConstantHeading(new Vector2d(22, -41), Math.toRadians(-120))
                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .lineToX(47)
                .strafeToConstantHeading(new Vector2d(47,-43))
                //                        .splineToConstantHeading(new Vector2d(-46,-35),Math.toRadians(-90))
//                        .splineToConstantHeading(new Vector2d(-36,-64),Math.toRadians(-90))
//                        .lineToY(-46)
//                         .splineToConstantHeading(new Vector2d(-24.4, -35), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-36,-60),Math.toRadians(90))
                        .turn(Math.toRadians(90))
//TODO                RED LEFT MIDDLE
                .lineToY(-35)
                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .lineToX(47)
                .strafeToConstantHeading(new Vector2d(47,-34))
                .waitSeconds(1)

                .splineToConstantHeading(new Vector2d(-36,-60),Math.toRadians(90))
                .turn(Math.toRadians(90))
// TODO                RED LEFT LEFT
                .splineToConstantHeading(new Vector2d(-45, -41), Math.toRadians(120))
                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .lineToX(47)
                .strafeToConstantHeading(new Vector2d(47,-29))
                .splineToConstantHeading(new Vector2d(-36,-60),Math.toRadians(90))
                .turn(Math.toRadians(90))
//TODO                RED LEFT RIGHT
                .lineToY(-37)
                .turn(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(-32, -35), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-36,-35), Math.toRadians(45))
                .turn(Math.toRadians(45))
                .lineToY(-58)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}