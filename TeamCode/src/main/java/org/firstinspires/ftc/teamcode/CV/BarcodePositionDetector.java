package org.firstinspires.ftc.teamcode.CV;

import android.preference.SwitchPreference;

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
        NOT_READ
    }

    private BarcodePosition barcodePosition = BarcodePosition.NOT_READ;

	/*
    static final Rect LEFT_ROI = new Rect(
            new Point( 0, 70 ),
            new Point( 125, 160 ) );
    static final Rect MIDDLE_ROI = new Rect(
            new Point( 125, 70 ),
            new Point( 250, 160 ) );
    static final Rect RIGHT_ROI = new Rect(
            new Point( 250, 70 ),
            new Point( 320, 160 ) );
	*/

    //static double PERCENT_COLOR_THRESHOLD = 0.02;
	
	public static void saveMat(Mat img, String path)
	{
		Imgcodecs.imwrite(path, img);
	}

    public BarcodePositionDetector( Telemetry t ) {
        telemetry = t;
    }

    public boolean isBlack(Mat frame)
    {
        double threshold = 0.05;
        threshold *= threshold;
        double norm = 0;
        Scalar mean = Core.mean(frame);
        for (double d : mean.val)
            norm += d * d;
        return norm < threshold;
    }
    //public Mat processFrame( Mat input, String type ) {
	public Mat processFrame(Mat input)
	{
		//String time = "" + System.nanoTime();
		//String dir = "/sdcard/FIRST/";
		//saveMat(input, dir + time + ".png");
        if (isBlack(input)) {
            barcodePosition = BarcodePosition.NOT_READ;
            return input;
        }

		ArrayList<Mat> channels = new ArrayList<Mat>();
        //chNgwe
//      Imgproc.cvtColor( input, mat,  Imgproc.COLOR_RGB2HSV_FULL);

		/*
		EmergencyLogger logger = new EmergencyLogger();
		logger.write("sizeof(input) = %s", input.total());
        logger.write("||input||^2 = %s", input.dot(input));
		logger.close();

        Scalar lowHSV;
        Scalar highHSV;
		*/

        Core.split(input, channels);
		//for (int i = 0; i < channels.size(); ++i)
			//saveMat(channels.get(i), dir + time + "-" + i + ".png");

       /* if( type.equalsIgnoreCase( "duck" ) ) {
            lowHSV = new Scalar(20, 100, 100);//25, 25, 35
            highHSV = new Scalar(30, 255, 255);
        } else {
            lowHSV = new Scalar( 40, 50, 70 );
            highHSV = new Scalar( 65, 255, 255 );
        } */
 /*       Core.inRange( mat, lowHSV, highHSV, mat );

        Mat left = mat.submat( LEFT_ROI );
        Mat middle = mat.submat( MIDDLE_ROI );
        Mat right = mat.submat( RIGHT_ROI );

        double leftValue = Core.sumElems( left ).val[0] / LEFT_ROI.area( ) / 255;
        double middleValue = Core.sumElems( middle ).val[0] / MIDDLE_ROI.area( ) / 255;
        double rightValue = Core.sumElems( right ).val[0] / RIGHT_ROI.area( ) / 255;

        left.release( );
        middle.release( );
        right.release( );
*/

        Scalar rAvg = Core.mean(channels.get(0));
        Scalar gAvg = Core.mean(channels.get(1));
        Scalar bAvg = Core.mean(channels.get(2));
        // one dot left, three dot right
		// red = 3 dots = right, green = 2 dots = middle, blue = 1 dot = left
		// FIXME: temporarily scrambled colors -> dots for testing
        boolean leftBool = bAvg.val[0] > gAvg.val[0] && bAvg.val[0] > rAvg.val[0];
        boolean middleBool = gAvg.val[0] > bAvg.val[0] && gAvg.val[0] > rAvg.val[0];
        boolean rightBool = rAvg.val[0] > bAvg.val[0] && rAvg.val[0] > gAvg.val[0];

        /*
		try {
			FileWriter r = new FileWriter(new File("/sdcard/FIRST/emergency.log\n"));
			r.write(String.format(
						"r=%s, g=%s, b=%s, left=%s, mid=%s, right=%s, len=%s, len[0]=%s",
						rAvg, gAvg, bAvg,
						leftBool, middleBool, rightBool,
						channels.size(), channels.get(0).total()));
			r.close();
		} catch (Exception e) {}
		*/

        if( rightBool ) {
            barcodePosition = BarcodePosition.RIGHT;
            telemetry.addData( "Location", "right" );
        } else if( leftBool ) {
            barcodePosition = BarcodePosition.LEFT;
            telemetry.addData( "Location", "left" );
        } else if( middleBool ) {
            barcodePosition = BarcodePosition.MIDDLE;
            telemetry.addData( "Location", "middle" );
        } else {
            barcodePosition = BarcodePosition.NOT_FOUND;
            telemetry.addData( "Location", "not found" );
        }
    //    Imgproc.cvtColor( mat, mat, Imgproc.COLOR_GRAY2RGB );

        //Scalar elementColor = new Scalar( 22, 203, 172 );
		//Scalar notElement = new Scalar( 0, 255, 0 );

   //     Imgproc.rectangle( mat, LEFT_ROI, barcodePosition == BarcodePosition.LEFT ? notElement : elementColor );
     //   Imgproc.rectangle( mat, RIGHT_ROI, barcodePosition == BarcodePosition.RIGHT ? notElement : elementColor );
       // Imgproc.rectangle( mat, MIDDLE_ROI, barcodePosition == BarcodePosition.MIDDLE ? notElement : elementColor );
		return input;
        //return mat;
    }

	/*
    @Override
    public Mat processFrame( Mat input ) {
		String path = "/sdcard/FIRST/cvtest-" + System.nanoTime() + ".png";
		Imgcodecs.imwrite(path, input);
        Mat elementImage = processFrame( input, "element" );
        Mat duckImage = processFrame( input, "duck" );
        double eleValue = Core.sumElems( elementImage ).val[0] / (elementImage.rows( ) * elementImage.cols( )) / 255;
        double duckValue = Core.sumElems( duckImage ).val[0] / (duckImage.rows( ) * duckImage.cols( )) / 255;
        if( eleValue < duckValue )
            return duckImage;
        return elementImage;
    }
	*/

    public BarcodePosition getBarcodePosition( ) {
        return barcodePosition;
    }
}
