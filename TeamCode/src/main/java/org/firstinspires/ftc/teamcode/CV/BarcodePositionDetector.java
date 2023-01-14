package org.firstinspires.ftc.teamcode.CV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.EmergencyLogger;

import java.util.ArrayList;
import java.io.*;

public class BarcodePositionDetector extends OpenCvPipeline {

    Telemetry telemetry;

    public enum BarcodePosition {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND,
		NOT_READ,
    }

    private BarcodePosition barcodePosition = BarcodePosition.NOT_READ;

    static double BLACK_THRESHOLD = 0.05;
	static int imagesRead = 0;
	
	public static void saveMat(Mat img, String path)
	{
		Imgcodecs.imwrite(path, img);
	}

    public BarcodePositionDetector( Telemetry t ) {
        telemetry = t;
    }

	public boolean isBlack(Mat frame)
	{
		Scalar avg = Core.mean(frame);
		double norm = 0;
		for (double d : avg.val)
			norm += d * d;
		return norm < BLACK_THRESHOLD * BLACK_THRESHOLD;
	}

	public Mat processFrame(Mat input)
	{
		//String time = "" + System.nanoTime();
		//String dir = "/sdcard/FIRST/";
		//saveMat(input, dir + time + ".png");
		ArrayList<Mat> channels = new ArrayList<Mat>();
//      Imgproc.cvtColor( input, mat,  Imgproc.COLOR_RGB2HSV_FULL);

		if (isBlack(input)) {
			barcodePosition = barcodePosition.NOT_READ;
			telemetry.addData("Location", "too dark");
			return input;
		}

        Core.split(input, channels);
		for (int i = 0; i < channels.size(); ++i)
			saveMat(channels.get(i), dir + time + "-" + i + ".png");

        Scalar rAvg = Core.mean(channels.get(0));
        Scalar gAvg = Core.mean(channels.get(1));
        Scalar bAvg = Core.mean(channels.get(2));

        // one dot left, three dot right
		// red = 3 dots = right, green = 2 dots = middle, blue = 1 dot = left
		// FIXME: temporarily scrambled colors -> dots for testing
        boolean leftBool = bAvg.val[0] > gAvg.val[0] && bAvg.val[0] > rAvg.val[0];
        boolean middleBool = gAvg.val[0] > bAvg.val[0] && gAvg.val[0] > rAvg.val[0];
        boolean rightBool = rAvg.val[0] > bAvg.val[0] && rAvg.val[0] > gAvg.val[0];

        if (rightBool) {
            barcodePosition = BarcodePosition.RIGHT;
            telemetry.addData( "Location", "right" );
        } else if (leftBool) {
            barcodePosition = BarcodePosition.LEFT;
            telemetry.addData( "Location", "left" );
        } else if (middleBool) {
            barcodePosition = BarcodePosition.MIDDLE;
            telemetry.addData( "Location", "middle" );
        } else {
            barcodePosition = BarcodePosition.NOT_FOUND;
            telemetry.addData( "Location", "not found" );
        }
		return input;
    }

    public BarcodePosition getBarcodePosition() {
        return barcodePosition;
    }
}
