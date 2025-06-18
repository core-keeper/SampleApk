package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

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

    public Mat getMatImage() {
        return this.image;
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

    public ArucoResult aruco(String area) {
        // area: "area1", "area2", "area3", "area4"
        List<ArucoResult> arucoResults = aruco();

        if (arucoResults.isEmpty()) {
            Log.i("image_aruco", "未檢測到 Aruco 標記。");
            return null;
        }

        ArucoResult finalArucoResult = arucoResults.get(0);
        double finalPosition = finalArucoResult.corners.get(0, 0)[0];
        if (area.equals("area2")) {
            for (ArucoResult arucoResult: arucoResults) {
                Mat arucoCornerPointsMat = arucoResult.corners;
                if (arucoCornerPointsMat.get(0, 0)[0] < finalPosition) {
                    finalArucoResult = arucoResult;
                    finalPosition = arucoCornerPointsMat.get(0, 0)[0];
                }
            }
        } else if (area.equals("area3")) {
            for (ArucoResult arucoResult: arucoResults) {
                Mat arucoCornerPointsMat = arucoResult.corners;
                if (arucoCornerPointsMat.get(0, 0)[0] > finalPosition) {
                    finalArucoResult = arucoResult;
                    finalPosition = arucoCornerPointsMat.get(0, 0)[0];
                }
            }
        }
        return finalArucoResult;
    }

    public Image crop(String area) {
        // 首先，獲取經 A4 紙透視校正後的影像
        Image correctedA4Image = correctA4Paper(area);

        if (correctedA4Image == null) {
            Log.e("Image_crop", "無法獲取校正後的 A4 影像，無法執行裁剪。");
            return null;
        }

        Mat correctedMat = correctedA4Image.image; // 獲取校正後的底層 Mat

        // 定義裁剪區域：從左上角 (0,0) 開始，寬度 224px，高度 224px
        int cropX = 0;
        int cropY = 0;
        int cropWidth = 224;
        int cropHeight = 224;

        // 檢查裁剪區域是否超出影像邊界
        if (cropX + cropWidth > correctedMat.cols() || cropY + cropHeight > correctedMat.rows()) {
            Log.w("Image_crop", "裁剪區域 (" + cropWidth + "x" + cropHeight + ") 超出校正後影像的邊界 (" + correctedMat.cols() + "x" + correctedMat.rows() + ")。將調整裁剪尺寸。");
            // 如果超出，將裁剪尺寸限制在影像尺寸內
            cropWidth = Math.min(cropWidth, correctedMat.cols() - cropX);
            cropHeight = Math.min(cropHeight, correctedMat.rows() - cropY);
            if (cropWidth <= 0 || cropHeight <= 0) {
                Log.e("Image_crop", "調整後的裁剪區域無效，無法進行裁剪。返回未裁剪的校正影像。");
                return correctedA4Image; // 返回未裁剪但已校正的影像
            }
        }

        // 創建一個 Rect 物件定義裁剪區域
        org.opencv.core.Rect roi = new org.opencv.core.Rect(cropX, cropY, cropWidth, cropHeight);

        // 使用 submat() 方法進行裁剪
        Mat croppedMat = new Mat(correctedMat, roi);
        Log.i("Image_crop", "影像已成功裁剪為 " + croppedMat.cols() + "x" + croppedMat.rows() + " 像素。");

        // 返回裁剪後的影像
        return new Image(croppedMat, api);
    }

    public void save(String imageName) {
        api.saveMatImage(image, imageName);
    }

    /**
     * 根據Aruco標記進行位置修正，不調整姿態。
     * 目標：將Aruco的左下角對齊圖像中心。
     * 位置修正的位移將被限制在垂直於 Astrobee 當前姿態所定義的法向量的平面上。
     * @param api KiboRpcApi 實例。
     * @param axisFrame 圖像的法向量。
     * @param area "area1", "area2", "area3", "area4" 其中一個
     * @param frequency 次數
     * @return Result 物件，表示移動操作的成功或失敗。
     */
    public static Frame anchor(KiboRpcApi api, Frame axisFrame, String area, int frequency) {
        Frame anchorFrame = null;

        for (int i = 0; i < frequency; i++) {
            Frame currentFrame = new Frame(api);

            // 1. 獲取當前圖像 (建議先去畸變)
            Image currentImage = Image.undistort(api); // 使用去畸變後的圖像進行 Aruco 檢測和定位

            // 2. 檢測 Aruco 標記
            ArucoResult arucoResult = currentImage.aruco(area);

            if (arucoResult == null) {
                Log.w("Image_Anchor", "未檢測到 Aruco 標記。不進行位置修正。");
                return null;
            } else {
                // 假設第一個檢測到的 Aruco 標記是用於錨定的
                Mat arucoCornerPointsMat = arucoResult.corners;

                // Aruco 標記的像素座標角點 (通常是左上[0], 右上[1], 右下[2], 左下[3])
                Point arucoBottomLeftPx = new Point(arucoCornerPointsMat.get(0, 3)[0], arucoCornerPointsMat.get(0, 3)[1]);
                Log.i("Image_Anchor", "檢測到的 Aruco 左下角像素點: (" + arucoBottomLeftPx.x + ", " + arucoBottomLeftPx.y + ")");

                // 3. 計算圖片中心點
                double imageCenterX = currentImage.image.cols() / 2.0;
                double imageCenterY = currentImage.image.rows() / 2.0;
                Log.i("Image_Anchor", "圖像中心像素點: (" + imageCenterX + ", " + imageCenterY + ")");

                // 4. 計算 Aruco 左下角與圖片中心點之間的像素偏差 (delta_x_px, delta_y_px)
                // 這裡的偏差是從圖像中心指向 Aruco 左下角的向量
                double deltaX_px = arucoBottomLeftPx.x - imageCenterX; // 圖像 X 軸偏差 (向右為正)
                double deltaY_px = arucoBottomLeftPx.y - imageCenterY; // 圖像 Y 軸偏差 (向下為正)
                Log.i("Image_Anchor", "像素偏差 (Aruco左下角 - 圖像中心): dx=" + deltaX_px + ", dy=" + deltaY_px);

                // 5. 將像素偏差轉換為公尺
                final double PIXELS_PER_METER = 567.0; // 您提供的像素轉公尺比例
                double deltaX_m = deltaX_px / PIXELS_PER_METER;
                double deltaY_m = deltaY_px / PIXELS_PER_METER;
                Log.i("Image_Anchor", "公尺偏差: dx_m=" + deltaX_m + " m, dy_m=" + deltaY_m + " m");

                // 6. Astrobee 世界座標系中的移動向量
                Vector delta = new Vector(0.0, 0.0, 0.0);
                Vector axis = axisFrame.getPosition();
                if (axis.getX() > 0.5 && axis.getY() < 0.5 && axis.getZ() < 0.5) {
                    // YZ 平面
                    delta = new Vector(0.0, -deltaX_m, deltaY_m);
                } else if (axis.getX() < 0.5 && axis.getY() > 0.5 && axis.getZ() < 0.5) {
                    // ZX 平面
                    delta = new Vector(deltaX_m, 0.0, deltaY_m);
                } else if (axis.getX() < 0.5 && axis.getY() < 0.5 && axis.getZ() > 0.5) {
                    // XY 平面
                    delta = new Vector(deltaY_m, deltaX_m, 0.0);
                }

                anchorFrame = new Frame(
                        currentFrame.getPosition().absolute(delta),
                        currentFrame.getOrientation()
                );
                Log.i("Image_Anchor", "Astrobee 的移動向量: " + anchorFrame);
                anchorFrame.moveTo(api, true);
            }
        }

        return anchorFrame;
    }

    public Image correctA4Paper(String area) {
        // area: "area1", "area2", "area3", "area4"
        ArucoResult arucoResult = aruco(area);
        return correctA4Paper(arucoResult);
    }

    public Image correctA4Paper(ArucoResult arucoResult) {
        if (arucoResult == null) {
            Log.i("image_correct", "未檢測到 Aruco 標記。無法校正 A4 紙。");
            return null;
        }

        Mat arucoCornerPointsMat = arucoResult.corners; // 假設第一個檢測到的標記是我們需要的

        // Aruco 標記的像素座標角點 (通常是左上, 右上, 右下, 左下)
        Point[] arucoDetectedCorners = new Point[4];
        for (int i = 0; i < 4; i++) {
            double[] xy = arucoCornerPointsMat.get(0, i);
            arucoDetectedCorners[i] = new Point(xy[0], xy[1]);
        }

        // --- 根據「最一開始的設定」和「A4 橫向」重新設定 ---

        // A4 紙的實際物理尺寸 - 假設為橫向 (重要！)
        double a4WidthCm = 29.7;          // 橫向 A4 的寬度
        double a4HeightCm = 21.0;         // 橫向 A4 的高度

        // Aruco 標記的實際物理尺寸 (5cm x 5cm)
        double arucoPhysicalSizeCm = 5.0;

        // 計算像素/厘米比例 (使用 Aruco 標記檢測到的像素大小)
        double arucoPixelWidth = Math.sqrt(Math.pow(arucoDetectedCorners[1].x - arucoDetectedCorners[0].x, 2) + Math.pow(arucoDetectedCorners[1].y - arucoDetectedCorners[0].y, 2));
        double arucoPixelHeight = Math.sqrt(Math.pow(arucoDetectedCorners[3].x - arucoDetectedCorners[0].x, 2) + Math.pow(arucoDetectedCorners[3].y - arucoDetectedCorners[0].y, 2));

        double pixelsPerCm = (arucoPixelWidth + arucoPixelHeight) / (2 * arucoPhysicalSizeCm);

        if (pixelsPerCm == 0 || Double.isNaN(pixelsPerCm) || Double.isInfinite(pixelsPerCm)) {
            Log.i("image_correct", "無法計算像素-厘米比，可能是Aruco標記太小或未正確檢測。");
            return null;
        }

        // Aruco 標記的左上角相對於「橫向 A4 紙」左上角的物理距離 (以厘米為單位)
        double arucoTL_X_from_A4TL_cm = 22.1;
        double arucoTL_Y_from_A4TL_cm = 4.25;

        // --- 最關鍵的步驟：利用 Aruco 的透視變換來推算 A4 紙的真實角點 ---

        // 1. 定義 Aruco 標記在「其自身物理座標系」中的四個角點
        // Aruco 的物理座標系：左上角為 (0,0)
        Point arucoLocalTopLeftCm     = new Point(0, 0);
        Point arucoLocalTopRightCm    = new Point(arucoPhysicalSizeCm, 0);
        Point arucoLocalBottomRightCm = new Point(arucoPhysicalSizeCm, arucoPhysicalSizeCm);
        Point arucoLocalBottomLeftCm  = new Point(0, arucoPhysicalSizeCm);

        // 2. 定義 A4 紙的四個角點在「Aruco 標記的物理座標系」中的位置
        // 這些是「A4 紙的角點」相對於「Aruco 標記的左上角」的物理偏移
        // 注意：A4 紙是橫向的 (29.7cm x 21.0cm)，且 ArucoTL_X/Y_from_A4TL_cm 是 Aruco 距離 A4 左上角的偏移
        Point a4LocalTopLeftCm = new Point(
                -arucoTL_X_from_A4TL_cm,
                -arucoTL_Y_from_A4TL_cm
        );
        Point a4LocalTopRightCm = new Point(
                a4WidthCm - arucoTL_X_from_A4TL_cm,
                -arucoTL_Y_from_A4TL_cm
        );
        Point a4LocalBottomRightCm = new Point(
                a4WidthCm - arucoTL_X_from_A4TL_cm,
                a4HeightCm - arucoTL_Y_from_A4TL_cm
        );
        Point a4LocalBottomLeftCm = new Point(
                -arucoTL_X_from_A4TL_cm,
                a4HeightCm - arucoTL_Y_from_A4TL_cm
        );

        // 3. 計算從「Aruco 物理座標系」到「圖像像素座標系」的透視變換矩陣
        MatOfPoint2f arucoSourceCm = new MatOfPoint2f(
                arucoLocalTopLeftCm,
                arucoLocalTopRightCm,
                arucoLocalBottomRightCm,
                arucoLocalBottomLeftCm
        );

        MatOfPoint2f arucoDestPixels = new MatOfPoint2f(
                arucoDetectedCorners[0], // Aruco 的左上角 (像素)
                arucoDetectedCorners[1], // Aruco 的右上角 (像素)
                arucoDetectedCorners[2], // Aruco 的右下角 (像素)
                arucoDetectedCorners[3]  // Aruco 的左下角 (像素)
        );

        Mat arucoToImagePerspectiveTransform = Imgproc.getPerspectiveTransform(arucoSourceCm, arucoDestPixels);

        // 4. 使用這個變換矩陣來計算「A4 紙的四個角點」在「圖像像素空間」中的位置
        MatOfPoint2f a4SourceCm = new MatOfPoint2f(
                a4LocalTopLeftCm,
                a4LocalTopRightCm,
                a4LocalBottomRightCm,
                a4LocalBottomLeftCm
        );

        MatOfPoint2f a4CornersInImagePixels = new MatOfPoint2f();
        // 將 A4 的物理點通過 Aruco 的變換矩陣映射到圖像像素座標
        Core.perspectiveTransform(a4SourceCm, a4CornersInImagePixels, arucoToImagePerspectiveTransform);

        // 現在 a4CornersInImagePixels 包含了 A4 紙的四個角點在圖像中的像素位置，
        // 並且它們的順序應該是「左上、右上、右下、左下」，這正是 warpPerspective 所期望的源點順序。
        Point[] finalSourcePointsArray = a4CornersInImagePixels.toArray();

        MatOfPoint2f sourcePoints = new MatOfPoint2f(
                finalSourcePointsArray[0], // A4 的左上角
                finalSourcePointsArray[1], // A4 的右上角
                finalSourcePointsArray[2], // A4 的右下角
                finalSourcePointsArray[3]  // A4 的左下角
        );

        // --- 除錯視覺化：在圖像上繪製計算出的 A4 角點 ---
        Mat debugImage = image.clone();

        // 定義顏色 (BGR 格式, 0-255)
        Scalar blue = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);
        Scalar red = new Scalar(0, 0, 255);
        Scalar cyan = new Scalar(255, 255, 0);

        int radius = 10; // 圓圈半徑
        int thickness = 3; // 線條和圓圈的粗細

        // 在計算出的 A4 角點處繪製圓圈
        Imgproc.circle(debugImage, finalSourcePointsArray[0], radius, blue, thickness);
        Imgproc.circle(debugImage, finalSourcePointsArray[1], radius, green, thickness);
        Imgproc.circle(debugImage, finalSourcePointsArray[2], radius, red, thickness);
        Imgproc.circle(debugImage, finalSourcePointsArray[3], radius, cyan, thickness);

        // 繪製連接 A4 角點的線條 (應該形成一個四邊形)
        Imgproc.line(debugImage, finalSourcePointsArray[0], finalSourcePointsArray[1], blue, thickness);
        Imgproc.line(debugImage, finalSourcePointsArray[1], finalSourcePointsArray[2], green, thickness);
        Imgproc.line(debugImage, finalSourcePointsArray[2], finalSourcePointsArray[3], red, thickness);
        Imgproc.line(debugImage, finalSourcePointsArray[3], finalSourcePointsArray[0], cyan, thickness);

        // 保存除錯圖像，查看計算出的角點落在何處
        api.saveMatImage(debugImage, "debug_a4_corners.png");
        Log.i("image_correct", "已保存帶有計算出的 A4 角點的除錯圖像: debug_a4_corners.png");

        // --- 最終的透視變換 ---

        // 目標尺寸的定義變更：將短邊固定為 224px
        final int SHORT_SIDE_PX = 224;

        // A4 紙是橫向的：寬度 29.7cm (長邊), 高度 21.0cm (短邊)
        // 因此，目標圖像的高度 (短邊) 應為 SHORT_SIDE_PX
        int targetHeightPx = SHORT_SIDE_PX;

        // 根據短邊和 A4 紙的長寬比計算長邊 (寬度)
        double aspectRatio = a4WidthCm / a4HeightCm; // 29.7 / 21.0
        int targetWidthPx = (int) Math.round(targetHeightPx * aspectRatio);


        MatOfPoint2f destinationPoints = new MatOfPoint2f(
                new Point(0, 0),
                new Point(targetWidthPx - 1, 0),
                new Point(targetWidthPx - 1, targetHeightPx - 1),
                new Point(0, targetHeightPx - 1)
        );

        Mat perspectiveTransform = Imgproc.getPerspectiveTransform(sourcePoints, destinationPoints);

        Mat correctedMat = new Mat();
        Imgproc.warpPerspective(image, correctedMat, perspectiveTransform, new Size(targetWidthPx, targetHeightPx));

        // 返回校正後的影像，不在此處進行裁剪
        return new Image(correctedMat, api);
    }
}
