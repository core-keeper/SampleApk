package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ORBObjectDetector extends ObjectDetector {
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

    public ORBObjectDetector(Context context) {
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
            Imgproc.cvtColor(inputMat, grayScene, Imgproc.COLOR_BGR2GRAY);
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
