package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import java.util.Arrays;


@Autonomous(name = "Find Skystones Share")
public class VuforiaButNot extends LinearOpMode {
    int colorSide = 1;
    private VuforiaLocalizer vuforia;
    boolean farTest, centerTest, closeTest;
    int testxStart = 351, testyStart = 187, testwidth = 722, testheight = 120;

//Telemetry outputs the average red value, average green value, average blue value,
//number of yellow pixels, and number of black pixels.
//we found the number of yellow pixels to be more accurate so that is what this
//file does.
//in order to run you need to define the pixel locations of the blocks... the pixels
//must be measured from the bottom right pixel, though phone orientation may affect this
//the test width is for the entire 3 block row. We found the best way to find these
//measurements by opening up the built in Concept: VuMark Id and using a ruler to measure
//the distance from the bottom right corner. Then use the entire distance across the view
//to get the total x distance and using the fact that the camera is 100x720 to get
//the partial x distance. This is repeated to get every number and then fine tuned.

    public void runOpMode() throws InterruptedException {
        initVuforia();

        waitForStart();
        while(opModeIsActive()) {
            findSkystone();
            telemetry.update();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AaIK0f//////AAAAGTsgmHszM030skFBvcnAJlNSWaH7oKLZhxAZeCi7ToBGSKkO7T3EvzsRVYQdyDp2X+TFK6TQs+3WoCHkZXDYPQd87f77D6kvcBr8zbJ07Fb31UKiXdUBvX+ZQSV3kBhdAoxhfMa0WPgys7DYaeiOmM49CsNra7nVh05ls0th3h07wwHz3s/PBZnQwpbfr260CDgqBv4e9D79Wg5Ja5p+HAOJvyqg2r/Z5dOyRvVI3f/jPBRZHvDgDF9KTcuJAPoDHxfewmGFOFtiUamRLvcrkK9rw2Vygi7w23HYlzFO7yap+jUk1bv0uWNc0j5HPJDAjqa2ijBN9aVDrxzmFJml5WMA3GJJp8WOd9gkGhtI/BIo";
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.setFrameQueueCapacity(1);
        vuforia.enableConvertFrameToBitmap();
    }

    public void findSkystone() throws InterruptedException {

        farTest = false;
        centerTest = false;
        closeTest = false;

        Bitmap bitmap = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().take());
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        int[] rawColorArray = new int[bitmap.getWidth() * bitmap.getHeight()];
        bitmap.getPixels(rawColorArray, 0, width, 0, 0, width, height);
        double[][][] pixels = new double[width][height][3];
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                //telemetry.addLine("width: " + i + " height: " + j + " value: "+((rawColorArray[j*width+i])&0xFF));
                pixels[x][y][0] = ((rawColorArray[y * width + x] >> 16) & 0xFF);
                pixels[x][y][1] = ((rawColorArray[y * width + x] >> 8) & 0xFF);
                pixels[x][y][2] = ((rawColorArray[y * width + x]) & 0xFF);
            }
        }
        int increment = testwidth / 3;
        double[] block1Test = averageValues(pixels, testxStart + 10, increment - 20, testyStart, testheight);
        double[] block2Test = averageValues(pixels, testxStart + increment + 10, increment - 20, testyStart, testheight);
        double[] block3Test = averageValues(pixels, testxStart + increment * 2 + 10, increment - 20, testyStart, testheight);
        //r,g,b,yellows,blacks
        telemetry.addLine(Arrays.toString(block1Test));
        telemetry.addLine(Arrays.toString(block2Test));
        telemetry.addLine(Arrays.toString(block3Test));

        if(block1Test[3] < block2Test[3] && block1Test[3] < block3Test[3]) {
            if(colorSide == 1) {
                closeTest = true;
            }
            else {
                farTest = true;
            }
        }
        else if(block2Test[3] < block1Test[3] && block2Test[3] < block3Test[3]) {
            if(colorSide == 1) {
                centerTest = true;
            }
            else {
                centerTest = true;
            }
        }
        else if(block3Test[3] < block2Test[3] && block3Test[3] < block1Test[3]) {
            if(colorSide == 1) {
                farTest = true;
            }
            else {
                closeTest = true;
            }
        }
        telemetry.addLine(("close " + closeTest) + ("   center " + centerTest) + ("    far " + farTest));
        telemetry.update();
    }

    public double[] averageValues(double[][][] pixels, int xStart, int xLength, int yStart, int yLength) {
        double[] output = new double[5];
        int yellows = 0;
        int blacks = 0;


        double numPixels = xLength * yLength;
        for(int j = yStart; j <= (yStart + yLength); j++) {
            for(int i = xStart; i <= (xStart + xLength); i++) {
                //telemetry.addLine("j: " + j + " i: " + i + "Values: " + pixels[i][j][0] +","+ pixels[i][j][1]+","+ pixels[i][j][2]);
                output[0] += pixels[i][j][0];
                output[1] += pixels[i][j][1];
                output[2] += pixels[i][j][2];
            }
        }
        output[0] /= numPixels;
        output[1] /= numPixels;
        output[2] /= numPixels;
        output[0] = Math.round(output[0]);
        output[1] = Math.round(output[1]);
        output[2] = Math.round(output[2]);

        for(int j = yStart; j <= (yStart + yLength); j++) {
            for(int i = xStart; i <= (xStart + xLength); i++) {
                //telemetry.addLine("j: " + j + " i: " + i + "Values: " + pixels[i][j][0] +","+ pixels[i][j][1]+","+ pixels[i][j][2]);
                if(pixels[i][j][0] > 90 && pixels[i][j][1] > 90 && pixels[i][j][2] < 120 && pixels[i][j][0] + pixels[i][j][1] > pixels[i][j][2] * 2.7) {
                    yellows++;
                }
                if(pixels[i][j][0] < 100 && pixels[i][j][1] < 100 && pixels[i][j][2] < 100 && (pixels[i][j][0] + pixels[i][j][1] + pixels[i][j][2]) / 3 < 75) {
                    blacks++;
                }
            }
        }
        output[3] = yellows;
        output[4] = blacks;
        return output;
    }
}
