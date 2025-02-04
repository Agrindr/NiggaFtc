package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import javax.imageio.ImageIO;

public class MeepMeepViz {
    public static void main(String[] args) {
        Image image = null;
        try {
            String projectPath = System.getProperty("user.dir");
            Path imagePath = Paths.get(projectPath, "MeepMeep/src/main/java/com/example/meepmeep/FTCFIELDJANCLK.png");

            // Print the project directory
            System.out.println("Current project path: " + imagePath.toString());

            File imageFile = new File(imagePath.toString());


            image = ImageIO.read(imageFile);

            System.out.println("Successfully loaded image!!");
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Error loading image.");
        }


        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .forward(30)
                                .turn(Math.toRadians(90))
                                .build()
                );

        if (image == null) {
            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();

        }else {
            meepMeep.setBackground(image)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)

                    .start();
        }

    }
}