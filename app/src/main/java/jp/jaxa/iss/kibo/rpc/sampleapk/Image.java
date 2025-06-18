package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

public class Image extends Mat {
    private Mat image;
    private KiboRpcApi api;

    public Image(Mat image, KiboRpcApi api) {
        this.image = image;
        this.api = api;
    }

    public Image(KiboRpcApi api) {
        this(api.getMatNavCam(), api);
    }

    public static Image undistort(KiboRpcApi api) {
        Mat image = api.getMatNavCam();

        Mat cameraMatrix = new Mat(3,3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);

        Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        Mat undistortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);

        return new Image(undistortImg, api);
    }

    public List<ArucoResult> aruco() {
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, ids);
        List<ArucoResult> results = new ArrayList<>();

        for (int i = 0; i < corners.size(); i++) {
            results.add(new ArucoResult(corners.get(i), ids.get(i, 0)[0]));
        }

        return results;
    }

    public Image crop() {
        List<ArucoResult> arucoResults = aruco();
        return correctA4Paper(arucoResults);
    }

    public void save(String imageName) {
        api.saveMatImage(image, imageName);
    }

    Image correctA4Paper(List<ArucoResult> arucoResults) {
        if (arucoResults.isEmpty()) {
            Log.i("image_correct", "未檢測到 Aruco 標記。無法校正 A4 紙。");
            return null;
        }

        Mat arucoCornerPointsMat = arucoResults.get(0).corners; // 假設第一個檢測到的標記是我們需要的

        // Aruco 標記的像素座標角點 (左上, 右上, 右下, 左下)
        Point[] arucoPointsArray = new Point[4];
        for (int i = 0; i < 4; i++) {
            double[] xy = arucoCornerPointsMat.get(0, i);
            arucoPointsArray[i] = new Point(xy[0], xy[1]);
        }

        // --- 根據「最一開始的設定」和「A4 橫向」重新設定 ---

        // A4 紙的實際物理尺寸 - 假設為橫向 (重要！)
        double a4WidthCm = 29.7;          // 橫向 A4 的寬度
        double a4HeightCm = 21.0;         // 橫向 A4 的高度

        // Aruco 標記的實際物理尺寸 (5cm x 5cm)
        double arucoPhysicalSizeCm = 5.0;

        // 計算像素/厘米比例 (使用 Aruco 標記檢測到的像素大小)
        double arucoPixelWidth = Math.sqrt(Math.pow(arucoPointsArray[1].x - arucoPointsArray[0].x, 2) + Math.pow(arucoPointsArray[1].y - arucoPointsArray[0].y, 2));
        double arucoPixelHeight = Math.sqrt(Math.pow(arucoPointsArray[3].x - arucoPointsArray[0].x, 2) + Math.pow(arucoPointsArray[3].y - arucoPointsArray[0].y, 2));

        double pixelsPerCm = (arucoPixelWidth + arucoPixelHeight) / (2 * arucoPhysicalSizeCm);

        if (pixelsPerCm == 0 || Double.isNaN(pixelsPerCm) || Double.isInfinite(pixelsPerCm)) {
            Log.i("image_correct", "無法計算像素-厘米比，可能是Aruco標記太小或未正確檢測。");
            return null;
        }

        // --- 使用您最一開始提供的相對位置 ---
        // Aruco 標記的左上角 (arucoPointsArray[0]) 相對於「橫向 A4 紙」左上角的物理距離 (以厘米為單位)
        // 這是您最初提供的值，現在明確應用於橫向 A4
        double arucoTL_X_from_A4TL_cm = 22.1;
        double arucoTL_Y_from_A4TL_cm = 4.25;

        // 計算 A4 紙的左上角像素位置
        // 從 Aruco 標記的左上角像素位置 (arucoPointsArray[0]) 向左和向上偏移
        Point a4TopLeft = new Point(
                arucoPointsArray[0].x - (arucoTL_X_from_A4TL_cm * pixelsPerCm),
                arucoPointsArray[0].y - (arucoTL_Y_from_A4TL_cm * pixelsPerCm)
        );

        // 根據橫向 A4 紙的尺寸 (29.7cm x 21.0cm) 和計算出的 pixelsPerCm，計算其他三個角點
        Point a4TopRight = new Point(
                a4TopLeft.x + (a4WidthCm * pixelsPerCm), // 橫向 A4 的寬度 (29.7cm)
                a4TopLeft.y
        );
        Point a4BottomRight = new Point(
                a4TopLeft.x + (a4WidthCm * pixelsPerCm), // 橫向 A4 的寬度 (29.7cm)
                a4TopLeft.y + (a4HeightCm * pixelsPerCm) // 橫向 A4 的高度 (21.0cm)
        );
        Point a4BottomLeft = new Point(
                a4TopLeft.x,
                a4TopLeft.y + (a4HeightCm * pixelsPerCm) // 橫向 A4 的高度 (21.0cm)
        );
        // --- 修正結束 ---


        MatOfPoint2f sourcePoints = new MatOfPoint2f(
                a4TopLeft,
                a4TopRight,
                a4BottomRight,
                a4BottomLeft
        );

        final int TARGET_WIDTH_PX = 224; // 目標寬度為 224px

        // 計算目標高度，保持橫向 A4 紙的長寬比 (29.7cm x 21.0cm)
        double aspectRatio = a4HeightCm / a4WidthCm; // 21.0 / 29.7
        int targetHeightPx = (int) Math.round(TARGET_WIDTH_PX * aspectRatio);

        MatOfPoint2f destinationPoints = new MatOfPoint2f(
                new Point(0, 0),
                new Point(TARGET_WIDTH_PX - 1, 0),
                new Point(TARGET_WIDTH_PX - 1, targetHeightPx - 1),
                new Point(0, targetHeightPx - 1)
        );

        Mat perspectiveTransform = Imgproc.getPerspectiveTransform(sourcePoints, destinationPoints);

        Mat correctedMat = new Mat();
        Imgproc.warpPerspective(image, correctedMat, perspectiveTransform, new Size(TARGET_WIDTH_PX, targetHeightPx));

        return new Image(correctedMat, api);
    }
}

// ArucoResult 類別保持不變
class ArucoResult {
    public Mat corners;
    public double id;

    public ArucoResult(Mat corners, double id) {
        this.corners = corners;
        this.id = id;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("ArucoResult{");
        sb.append("id=").append((int) id);

        if (corners != null && !corners.empty()) {
            sb.append(", corners=[");
            DecimalFormat df = new DecimalFormat("#.##");

            for (int i = 0; i < corners.cols(); i++) {
                double[] xy = corners.get(0, i);
                if (xy != null && xy.length >= 2) {
                    sb.append("(").append(df.format(xy[0])).append(", ").append(df.format(xy[1])).append(")");
                } else {
                    sb.append("invalid_corner");
                }
                if (i < corners.cols() - 1) {
                    sb.append(", ");
                }
            }
            sb.append("]");
        } else {
            sb.append(", corners=null");
        }
        sb.append("}");
        return sb.toString();
    }
}