package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

public class VanillaObjectDetector extends ObjectDetector {
    private final String TAG = this.getClass().getSimpleName();

    private final String[] TEMPLATE_FILE_NAMES = {
            "coin.png",
            "compass.png",
            "coral.png",
            "crystal.png",
            "diamond.png",
            "emerald.png",
            "fossil.png",
            "key.png",
            "letter.png",
            "shell.png",
            "treasure_box.png"
    };

    private final String[] TEMPLATE_NAMES = {
            "coin",
            "compass",
            "coral",
            "crystal",
            "diamond",
            "emerald",
            "fossil",
            "key",
            "letter",
            "shell",
            "treasure_box"
    };

    Mat[] templates;

    public VanillaObjectDetector(Context context) {
        // Load template images
        templates = new Mat[TEMPLATE_FILE_NAMES.length];
        for (int i = 0; i < TEMPLATE_FILE_NAMES.length; i++) {
            try{
                // Open the template image file in Bitmap from the file name and convert to Mat
                InputStream inputStream = context.getAssets().open(TEMPLATE_FILE_NAMES[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                // Convert the Bitmap to grayscale
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                // Assign to an array of templates
                templates[i] = mat;

                // Release the InputStream
                inputStream.close();

            } catch (IOException e) {
                Log.e(TAG, "Error loading template image: " + TEMPLATE_FILE_NAMES[i], e);
                e.printStackTrace();
            }
        }
    }

    @Override
    public List<ItemInfo> detect(Mat inputMat) {
        List<ItemInfo> items = new ArrayList<>();

        // Number of matches for each template
        int[] numMatches = new int[TEMPLATE_FILE_NAMES.length];

        // Get the number of template matches
        for (int tempNum = 0; tempNum < templates.length; tempNum++) {
            // number of matches
            int matchCount = 0;
            // Coordinates of the match location
            List<org.opencv.core.Point> matchLocations = new ArrayList<>();

            // Load the template image
            Mat template = templates[tempNum].clone();
            // Target image is the undistorted image
            Mat targetImg = inputMat.clone();

            // Pattern matching
            int widthMin = 20; //[px]
            int widthMax = 100; //[px]
            int changeWidth = 5; //[px]
            int changeAngle = 45; //[px]

            for (int size = widthMin; size <= widthMax; size += changeWidth) {
                for (int angle = 0; angle < 360; angle += changeAngle) {
                    // Resize the template image
                    Mat resizedTemplate = scalingresizeImage(template, size);
                    // Rotate the template image
                    Mat rotatedTemplate = rotImg(resizedTemplate, angle);

                    // Perform template matching
                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, rotatedTemplate, result, Imgproc.TM_CCOEFF_NORMED);

                    // Thresholding
                    double threshold = 0.7;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);

                    double maxVal = mmlr.maxVal;

                    if (maxVal >= threshold) {
                        // Create a mask for the detected region
                        Mat thresholdResult = new Mat();
                        Imgproc.threshold(result, thresholdResult, threshold, 1, Imgproc.THRESH_TOZERO);

                        // Get coordinates of the detected region
                        for (int y = 0; y < thresholdResult.rows(); y++) {
                            for (int x = 0; x < thresholdResult.cols(); x++) {
                                if (thresholdResult.get(y, x)[0] > 0) {
                                    matchLocations.add(new org.opencv.core.Point(x, y));
                                }
                            }
                        }

                        // Release resources
                        thresholdResult.release();
                    }

                    // Release resources
                    result.release();
                    rotatedTemplate.release();
                    resizedTemplate.release();
                }
            }

            List<org.opencv.core.Point> filteredMatches = removeDuplicates(matchLocations);
            matchCount += filteredMatches.size();

            // Number of matches
            numMatches[tempNum] = matchCount;

            // Release resources
            template.release();
            targetImg.release();
        }

        // Find the most matched template
        int mostMatchTemplateNum = getMxIndex(numMatches);

        // Store the results
        items.add(new ItemInfo(TEMPLATE_NAMES[mostMatchTemplateNum], numMatches[mostMatchTemplateNum]));

        return items;
    }

    private Mat scalingresizeImage(Mat image, int width) {
        int height = (int) ((double) image.rows() / image.cols() * width);
        // Create a new Mat object to hold the resized image
        Mat resizedImage = new Mat();

        // Resize the image using the specified width and height
        Imgproc.resize(image, resizedImage, new Size(width, height));

        return resizedImage;
    }

    private Mat rotImg(Mat img, int angle) {
        // Get the center of the image
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2, img.rows() / 2);

        // Create a rotation matrix
        Mat rotMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);

        // Create a new Mat object to hold the rotated image
        Mat rotatedImg = new Mat();

        // Rotate the image using the rotation matrix
        Imgproc.warpAffine(img, rotatedImg, rotMat, img.size());

        // Release resources
        rotMat.release();

        return rotatedImg;
    }

    private List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points) {
        double length = 10; // within 10 px
        List<org.opencv.core.Point> uniquePoints = new ArrayList<>();
        for (org.opencv.core.Point point : points) {
            boolean isIncluded = false;
            for (org.opencv.core.Point uniquePoint : uniquePoints) {
                double distance = calculateDistance(point, uniquePoint);

                if (distance <= length){
                    isIncluded = true;
                    break;
                }
            }

            if (!isIncluded) {
                uniquePoints.add(point);
            }
        }
        return uniquePoints;
    }

    private double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2) {
        // Calculate the distance between two points using the Euclidean distance formula
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }

    private int getMxIndex(int[] array){
        int max = 0;
        int maxIndex = 0;

        for (int i = 0; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}
