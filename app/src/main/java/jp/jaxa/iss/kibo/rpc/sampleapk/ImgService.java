package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import org.opencv.core.DMatch;
import org.opencv.core.Mat;


import android.util.Log;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;


import org.opencv.aruco.Dictionary;
import org.opencv.aruco.Aruco;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import java.io.InputStream;
import java.io.IOException;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import org.opencv.android.Utils;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Size;
import org.opencv.core.Core;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class ImgService extends KiboRpcService {

    // The TAG is used for logging.
    // You can use it to check the log in the Android Studio.
    private final String TAG = this.getClass().getSimpleName();

    // Type all the image in the template folder.
    //'coin.png', 'compass.png', 'coral.png', 'crystal.png',
    // 'diamond.png', 'emerald.png', 'fossil.png', 'key.png',
    // 'letter.png', 'shell.png', 'treasure_box.png']
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

    private final Vector[] AREA_POINTS = {
            new Vector(10.9d, -10.0000d, 5.195d),
            new Vector(10.925d, -8.875d, 4.602d),
            new Vector(10.925d, -7.925d, 4.60093d),
            new Vector(10.766d, -6.852d, 4.945d)
    };

    private final Quater[] AREA_QUATERNIONS = {
            new Quater(0f, 0f, -0.707f, 0.707f),
            new Quater(0f, 0.707f, 0f, 0.707f),
            new Quater(0f, 0.707f, 0f, 0.707f),
            new Quater(0f, 0f, 1f, 0f)
    };

    @Override
    protected void runPlan1(){
        // Log the start of the mission.
        Log.i(TAG, "Start mission");
        // The mission starts.
        api.startMission();

        // Prepare the variables to store treasure item locations
        String[] foundItems = new String[AREA_POINTS.length];
        int[] itemCounts = new int[AREA_POINTS.length];

        // Load template images
        Mat[] templates = new Mat[TEMPLATE_FILE_NAMES.length];
        for (int i = 0; i < TEMPLATE_FILE_NAMES.length; i++) {
            try{
                // Open the template image file in Bitmap from the file name and convert to Mat
                InputStream inputStream = getAssets().open(TEMPLATE_FILE_NAMES[i]);
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

        // Visit each area
        for (int areaId = 0; areaId < AREA_POINTS.length; areaId++) {
            // Move to the current area
            api.moveTo(AREA_POINTS[areaId], AREA_QUATERNIONS[areaId], false);

            // Get a camera image
            Mat image = api.getMatNavCam();
            // Save the image for debugging
            api.saveMatImage(image, "area_" + (areaId + 1) + ".png");

            /* ********************************************** */
            /* Start image recognition process */
            /* ********************************************** */
            // AR tag detection
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            Mat ids = new Mat();
            Aruco.detectMarkers(image, dictionary, corners, ids);

            // Undistort the image
            Mat cameraMatrix = new Mat(3,3,CvType.CV_64F);
            cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);

            Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
            cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
            cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

            Mat undistortedImf = new Mat();
            Calib3d.undistort(image, undistortedImf, cameraMatrix, cameraCoefficients);

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
                Mat targetImg = undistortedImf.clone();


                // ------------- BEGIN: 用 ORB+BFMatcher 進行多目標偵測 -------------
                ORB orb = ORB.create(500);
                BFMatcher matcher = BFMatcher.create(BFMatcher.BRUTEFORCE_HAMMING, false);

                List<MatOfKeyPoint> tplKeyPoints = new ArrayList<>();
                List<Mat> tplDescriptors = new ArrayList<>();
                for (int t = 0; t < templates.length; t++) {
                    Mat grayTpl = new Mat();
                    Imgproc.cvtColor(templates[t], grayTpl, Imgproc.COLOR_BGR2GRAY);
                    MatOfKeyPoint kp = new MatOfKeyPoint();
                    Mat desc = new Mat();
                    orb.detectAndCompute(grayTpl, new Mat(), kp, desc);
                    tplKeyPoints.add(kp);
                    tplDescriptors.add(desc);
                    grayTpl.release();
                }

                Mat grayScene = new Mat();
                Imgproc.cvtColor(undistortedImf, grayScene, Imgproc.COLOR_BGR2GRAY);
                MatOfKeyPoint sceneKp = new MatOfKeyPoint();
                Mat sceneDesc = new Mat();
                orb.detectAndCompute(grayScene, new Mat(), sceneKp, sceneDesc);

                matchLocations = new ArrayList<>();
                final int MIN_MATCHES = 10;
                Mat mask = Mat.ones(grayScene.size(), CvType.CV_8U);
                for (int t = 0; t < templates.length; t++) {
                    MatOfKeyPoint tplKp = tplKeyPoints.get(t);
                    Mat tplDesc = tplDescriptors.get(t);
                    int found = 0;
                    while (true) {
                        List<MatOfDMatch> knn = new ArrayList<>();
                        matcher.knnMatch(tplDesc, sceneDesc, knn, 2);
                        List<DMatch> good = new ArrayList<>();
                        for (MatOfDMatch matOfD : knn) {
                            DMatch[] d = matOfD.toArray();
                            if (d.length >= 2 && d[0].distance < 0.75 * d[1].distance) {
                                good.add(d[0]);
                            }
                        }
                        if (good.size() < MIN_MATCHES) break;

                        List<Point> ptsTpl = new ArrayList<>(), ptsScn = new ArrayList<>();
                        for (DMatch m : good) {
                            ptsTpl.add(tplKp.toList().get(m.queryIdx).pt);
                            ptsScn.add(sceneKp.toList().get(m.trainIdx).pt);
                        }
                        MatOfPoint2f mTpl = new MatOfPoint2f();
                        mTpl.fromList(ptsTpl);
                        MatOfPoint2f mScn = new MatOfPoint2f();
                        mScn.fromList(ptsScn);

                        Mat H = Calib3d.findHomography(mTpl, mScn, Calib3d.RANSAC, 5);
                        if (H.empty()) break;

                        Mat tplCorners = new MatOfPoint2f(
                                new Point(0,0),
                                new Point(templates[t].cols(),0),
                                new Point(templates[t].cols(), templates[t].rows()),
                                new Point(0, templates[t].rows())
                        );
                        MatOfPoint2f sceneCorners = new MatOfPoint2f();
                        Core.perspectiveTransform(tplCorners, sceneCorners, H);
                        Point[] scPts = sceneCorners.toArray();
                        double cx = 0, cy = 0;
                        for (Point p : scPts) { cx += p.x; cy += p.y; }
                        matchLocations.add(new Point(cx/4, cy/4));

                        MatOfPoint mop = new MatOfPoint();
                        mop.fromList(Arrays.asList(scPts));
                        Mat regionMask = Mat.zeros(grayScene.size(), CvType.CV_8U);
                        List<MatOfPoint> contours = Arrays.asList(mop);
                        Imgproc.fillPoly(regionMask, contours, new Scalar(255));
                        Core.subtract(mask, regionMask, mask);

                        Mat newScene = new Mat();
                        grayScene.copyTo(newScene, mask);
                        orb.detectAndCompute(newScene, new Mat(), sceneKp, sceneDesc);

                        tplCorners.release();
                        sceneCorners.release();
                        H.release();
                        regionMask.release();
                        newScene.release();

                        found++;
                    }
                }

//                for (Point loc : matchLocations) {
//                    int i = 0; // 可根據需求進一步判斷對應哪個模板
//                    ids.put(i);
//                }
//                // ------------- END: 用 ORB+BFMatcher 進行多目標偵測 -------------
//
//                corner.release();
            }

        }

        // Move to the astronaut for reporting
        Vector astronautPoint = new Vector(11.143, -6.7607, 4.9654);
        Quater astronautQuaternion = new Quater(0f, 0f, 0.707f, 0.707f);
        api.moveTo(astronautPoint, astronautQuaternion, false);
        api.reportRoundingCompletion();

        /*
         Get information about the target item from the astronaut
         Mat targetImage = api.getMatNavCam();
         api.saveMatImage(targetImage, "target_item.png");
         TODO: Implement logic to identify the target item from astronaut's information
         For now, let's assume the target is in the first area
         String targetItem = TEMPLATE_NAMES[0]; // Replace with actual target item identification
         Notify that we recognized the target item
        */

        api.notifyRecognitionItem();

        //Find which area contains the target item and move there
        int targetArea = 0; // Default to first area if not found
        for (int i = 0; i < foundItems.length; i++) {
        }

        // Move to the area with the target item
        api.moveTo(AREA_POINTS[targetArea], AREA_QUATERNIONS[targetArea], false);

        // Take a snapshot of the target item
        api.takeTargetItemSnapshot();

        // Release resources
        // targetImage.release();
        for (Mat template : templates) {
            if (template != null) {
                template.release();
            }
        }
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here.
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here.
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
        org.opencv.core.Point center = new org.opencv.core.Point(2 / img.cols(), img.rows() / 2);

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

