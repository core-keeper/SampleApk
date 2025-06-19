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
import org.opencv.imgproc.Imgproc;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.ORB;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ORBObjectDetector extends ObjectDetector {
    private final String TAG = this.getClass().getSimpleName();

    // 常數定義，提高可讀性和維護性
    private static final int ORB_FEATURES = 500; // ORB 檢測器要提取的特徵點數量
    private static final int MIN_GOOD_MATCHES = 10; // 執行單應性矩陣計算所需的最小良好匹配點數
    private static final double KNN_RATIO_THRESHOLD = 0.75; // k-NN 比率測試的閾值 (Lowe's ratio test)
    private static final double RANSAC_REPROJECTION_THRESHOLD = 5.0; // RANSAC 重新投影誤差閾值
    private static final double DUPLICATE_DISTANCE_THRESHOLD = 10.0; // 移除重複點的距離閾值 (像素)

    // 定義模板文件名稱和對應的物件名稱
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

    // 儲存已載入的模板圖像 (灰度圖)
    Mat[] templates;

    // ORB 特徵點檢測器和描述符匹配器
    private ORB orb;
    private BFMatcher matcher;

    // 儲存所有模板圖像的關鍵點和描述符，在建構子中預計算一次
    private List<MatOfKeyPoint> allTplKeyPoints;
    private List<Mat> allTplDescriptors;

    /**
     * 建構子，用於初始化模板圖像並預計算其 ORB 特徵點和描述符。
     *
     * @param context Android 應用程式的上下文，用於訪問資產。
     */
    public ORBObjectDetector(Context context) {
        // 初始化 ORB 特徵點檢測器和 BFMatcher 匹配器
        orb = ORB.create(ORB_FEATURES);
        matcher = BFMatcher.create(BFMatcher.BRUTEFORCE_HAMMING, false);

        templates = new Mat[TEMPLATE_FILE_NAMES.length];
        allTplKeyPoints = new ArrayList<>();
        allTplDescriptors = new ArrayList<>();

        // 載入模板圖像並預計算其關鍵點和描述符
        for (int i = 0; i < TEMPLATE_FILE_NAMES.length; i++) {
            try {
                // 從資產中打開模板圖像文件
                InputStream inputStream = context.getAssets().open(TEMPLATE_FILE_NAMES[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                // 釋放 Bitmap 資源以減少記憶體佔用
                bitmap.recycle();

                // 將圖像轉換為灰度圖
                // 確保輸入圖像在轉換前不是單通道，如果不是，則會拋出錯誤
                if (mat.channels() > 1) {
                    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);
                }
                // 如果已經是單通道，則不需要轉換

                // 儲存灰度模板圖像
                templates[i] = mat;

                // 計算模板的關鍵點和描述符
                MatOfKeyPoint kp = new MatOfKeyPoint();
                Mat desc = new Mat();
                orb.detectAndCompute(mat, new Mat(), kp, desc); // `mat` (灰度模板) 用於計算

                // 儲存計算出的關鍵點和描述符
                allTplKeyPoints.add(kp);
                allTplDescriptors.add(desc);

                // 關閉輸入流
                inputStream.close();

            } catch (IOException e) {
                Log.e(TAG, "Error loading template image: " + TEMPLATE_FILE_NAMES[i], e);
                e.printStackTrace();
            }
        }
    }

    /**
     * 執行物件偵測。對於每個模板，它會嘗試在輸入圖像中找到匹配項。
     *
     * @param inputMat 輸入圖像 (場景圖像)。
     * @return 偵測到的物件資訊列表。
     */
    @Override
    public List<ItemInfo> detect(Mat inputMat) {
        List<ItemInfo> items = new ArrayList<>();

        // 檢查輸入圖像是否有效
        if (inputMat == null || inputMat.empty()) {
            Log.e(TAG, "Input Mat is null or empty.");
            return items; // 返回空列表
        }

        // 將輸入圖像轉換為灰度圖，只執行一次
        Mat grayScene;
        // 修正：檢查 inputMat 的通道數，避免對灰度圖進行 BGR2GRAY 轉換
        if (inputMat.channels() > 1) {
            grayScene = new Mat();
            Imgproc.cvtColor(inputMat, grayScene, Imgproc.COLOR_BGR2GRAY);
        } else {
            // 如果 inputMat 已經是灰度圖 (1 通道)，直接使用它或複製一份
            grayScene = inputMat.clone();
        }

        // 儲存每個模板的匹配數量
        int[] numMatches = new int[TEMPLATE_FILE_NAMES.length];

        // 遍歷每個模板以進行匹配
        for (int tempNum = 0; tempNum < templates.length; tempNum++) {
            // 獲取當前模板的關鍵點和描述符
            MatOfKeyPoint tplKp = allTplKeyPoints.get(tempNum);
            Mat tplDesc = allTplDescriptors.get(tempNum);

            // 檢查模板描述符是否為空，避免崩潰
            if (tplDesc.empty()) {
                Log.w(TAG, "Template " + TEMPLATE_NAMES[tempNum] + " has no descriptors. Skipping.");
                numMatches[tempNum] = 0;
                continue;
            }

            // 用於多物件偵測的場景關鍵點和描述符 (初始狀態)
            MatOfKeyPoint sceneKp = new MatOfKeyPoint();
            Mat sceneDesc = new Mat();
            // 注意：這裡只在需要時（即首次或遮罩更新後）計算場景特徵點
            orb.detectAndCompute(grayScene, new Mat(), sceneKp, sceneDesc);

            // 檢查場景描述符是否為空
            if (sceneDesc.empty()) {
                Log.w(TAG, "Scene has no descriptors. Skipping template " + TEMPLATE_NAMES[tempNum] + ".");
                numMatches[tempNum] = 0;
                sceneKp.release();
                sceneDesc.release();
                continue;
            }

            List<org.opencv.core.Point> currentMatchLocations = new ArrayList<>();
            // 創建一個遮罩，最初全部為白色 (255)，表示所有區域都可被檢測
            Mat mask = new Mat(grayScene.size(), CvType.CV_8U, new Scalar(255));

            int foundCount = 0;
            // 迴圈檢測同一物件的多個實例
            while (true) {
                // 進行 k-NN 匹配
                List<MatOfDMatch> knnMatches = new ArrayList<>();
                matcher.knnMatch(tplDesc, sceneDesc, knnMatches, 2);

                List<DMatch> goodMatches = new ArrayList<>();
                // 執行 Lowe's ratio test 以篩選良好匹配
                for (MatOfDMatch matOfD : knnMatches) {
                    DMatch[] d = matOfD.toArray();
                    // 確保至少有兩個匹配點，並應用比率測試
                    if (d.length >= 2 && d[0].distance < KNN_RATIO_THRESHOLD * d[1].distance) {
                        goodMatches.add(d[0]);
                    }
                    matOfD.release(); // 釋放 MatOfDMatch 資源
                }
                knnMatches.clear(); // 清空列表

                // 如果良好匹配數量不足，則停止此模板的檢測
                if (goodMatches.size() < MIN_GOOD_MATCHES) {
                    break;
                }

                // 從良好匹配中提取關鍵點
                List<Point> ptsTpl = new ArrayList<>();
                List<Point> ptsScn = new ArrayList<>();
                for (DMatch m : goodMatches) {
                    ptsTpl.add(tplKp.toList().get(m.queryIdx).pt);
                    ptsScn.add(sceneKp.toList().get(m.trainIdx).pt);
                }

                // 將 Point 列表轉換為 MatOfPoint2f
                MatOfPoint2f mTpl = new MatOfPoint2f();
                mTpl.fromList(ptsTpl);
                MatOfPoint2f mScn = new MatOfPoint2f();
                mScn.fromList(ptsScn);

                Mat H = new Mat();
                try {
                    // 計算單應性矩陣 H，使用 RANSAC 濾除離群點
                    H = Calib3d.findHomography(mTpl, mScn, Calib3d.RANSAC, RANSAC_REPROJECTION_THRESHOLD);
                } catch (Exception e) {
                    Log.e(TAG, "Error calculating homography for template " + TEMPLATE_NAMES[tempNum], e);
                    H.release(); // 確保在異常情況下也釋放 H
                    break;
                }

                // 如果單應性矩陣為空，則表示沒有足夠的內點來計算，停止檢測
                if (H.empty()) {
                    H.release();
                    mTpl.release();
                    mScn.release();
                    break;
                }

                // 獲取模板的四個角點
                Mat tplCorners = new MatOfPoint2f(
                        new Point(0, 0),
                        new Point(templates[tempNum].cols(), 0),
                        new Point(templates[tempNum].cols(), templates[tempNum].rows()),
                        new Point(0, templates[tempNum].rows())
                );
                MatOfPoint2f sceneCorners = new MatOfPoint2f();
                // 將模板的角點透過單應性矩陣變換到場景圖像中
                Core.perspectiveTransform(tplCorners, sceneCorners, H);

                // 計算偵測到的物件中心點
                Point[] scPts = sceneCorners.toArray();
                double cx = 0, cy = 0;
                for (Point p : scPts) {
                    cx += p.x;
                    cy += p.y;
                }
                // 將中心點添加到當前匹配位置列表
                currentMatchLocations.add(new Point(cx / 4, cy / 4));
                foundCount++; // 增加找到的實例計數

                // 創建一個區域遮罩，用於將已偵測到的物件從主場景圖像中排除
                MatOfPoint mop = new MatOfPoint();
                mop.fromList(Arrays.asList(scPts));
                Mat regionMask = Mat.zeros(grayScene.size(), CvType.CV_8U); // 初始為黑色
                List<MatOfPoint> contours = new ArrayList<>();
                contours.add(mop);
                // 填充多邊形，將檢測到的區域在 regionMask 中標記為白色 (255)
                Imgproc.fillPoly(regionMask, contours, new Scalar(255));

                // 從主遮罩中減去已偵測到的區域，以便下次檢測只在剩餘區域進行
                Core.subtract(mask, regionMask, mask);

                // 基於更新後的遮罩，從原始灰度場景圖像中複製未被遮罩的區域
                Mat newScene = new Mat();
                grayScene.copyTo(newScene, mask);

                // 釋放舊的場景關鍵點和描述符
                sceneKp.release();
                sceneDesc.release();

                // 在新的、已遮罩的場景圖像上重新計算關鍵點和描述符
                sceneKp = new MatOfKeyPoint();
                sceneDesc = new Mat();
                orb.detectAndCompute(newScene, new Mat(), sceneKp, sceneDesc);

                // 檢查新的場景描述符是否為空，如果是，則表示沒有更多特徵點可供匹配
                if (sceneDesc.empty()) {
                    sceneKp.release(); // 釋放空的關鍵點
                    sceneDesc.release(); // 釋放空的描述符
                    newScene.release(); // 釋放新場景圖像
                    // 釋放迴圈內臨時 Mat
                    H.release();
                    mTpl.release();
                    mScn.release();
                    tplCorners.release();
                    sceneCorners.release();
                    mop.release();
                    regionMask.release();
                    break;
                }

                // 釋放迴圈內臨時 Mat 資源
                H.release();
                mTpl.release();
                mScn.release();
                tplCorners.release();
                sceneCorners.release();
                mop.release();
                regionMask.release();
                newScene.release(); // 釋放用於計算新場景的 Mat
            }

            // 釋放在模板循環外創建的場景特徵點和描述符
            sceneKp.release();
            sceneDesc.release();
            mask.release(); // 釋放最終的遮罩

            // 對當前模板的匹配位置進行去重
            // 注意：在基於遮罩的多物件偵測策略下，此步驟可能並非嚴格必要，
            // 因為每次找到的實例應當是獨特的。
            // 但為保留原邏輯，暫時保留。如果仍有問題，可考慮實作 NMS。
            List<org.opencv.core.Point> filteredMatches = removeDuplicates(currentMatchLocations);
            numMatches[tempNum] = filteredMatches.size();
        }

        // 釋放灰度場景圖像
        // 只有當 grayScene 是新創建的 Mat 時才需要釋放。
        // 如果它只是 inputMat 的 clone，那麼釋放 grayScene 也會釋放 inputMat 的底層數據。
        // 為了安全起見，如果 inputMat 不是由當前方法創建的，則不應該釋放它。
        // 這裡確保 grayScene 是新創建的才釋放。
        if (inputMat.channels() > 1) { // 只有在進行過轉換才需要釋放 grayScene
            grayScene.release();
        }


        // 找到匹配數量最多的模板
        int mostMatchTemplateNum = getMxIndex(numMatches);

        // 將結果儲存到 ItemInfo 列表中
        items.add(new ItemInfo(TEMPLATE_NAMES[mostMatchTemplateNum], numMatches[mostMatchTemplateNum]));

        return items;
    }

    /**
     * 移除重複的匹配點。在指定距離閾值內的點被視為重複。
     * 考慮到 ORB + Homography 的多物件偵測方式，此方法可能不再是最高效或最必要的，
     * 非最大抑制 (NMS) 對於重疊的邊界框可能是更好的選擇。
     *
     * @param points 原始匹配點列表。
     * @return 移除了重複點的列表。
     */
    private List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points) {
        List<org.opencv.core.Point> uniquePoints = new ArrayList<>();
        for (org.opencv.core.Point point : points) {
            boolean isIncluded = false;
            for (org.opencv.core.Point uniquePoint : uniquePoints) {
                double distance = calculateDistance(point, uniquePoint);

                if (distance <= DUPLICATE_DISTANCE_THRESHOLD) {
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

    /**
     * 計算兩個點之間的歐幾里得距離。
     *
     * @param p1 點 1。
     * @param p2 點 2。
     * @return 兩個點之間的距離。
     */
    private double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2) {
        return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
    }

    /**
     * 找到整數陣列中最大值的索引。
     *
     * @param array 輸入整數陣列。
     * @return 最大值的索引。
     */
    private int getMxIndex(int[] array) {
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

    /**
     * 釋放偵測器持有的所有 OpenCV Mat 資源。
     * 建議在偵測器不再使用時呼叫此方法，以避免記憶體洩漏。
     */
    public void release() {
        // 釋放所有模板圖像
        if (templates != null) {
            for (Mat template : templates) {
                if (template != null) {
                    template.release();
                }
            }
        }
        // 釋放所有模板的關鍵點和描述符
        if (allTplKeyPoints != null) {
            for (MatOfKeyPoint kp : allTplKeyPoints) {
                if (kp != null) {
                    kp.release();
                }
            }
            allTplKeyPoints.clear(); // 清空列表
        }
        if (allTplDescriptors != null) {
            for (Mat desc : allTplDescriptors) {
                if (desc != null) {
                    desc.release();
                }
            }
            allTplDescriptors.clear(); // 清空列表
        }
        // ORB 和 BFMatcher 的 release() 方法通常不要求，它們由 GC 管理
        // 但如果它們佔用大量原生資源且有明確的 release 方法，則在此處呼叫
        // 在 OpenCV Java 中，通常不需要顯式釋放 ORB 和 BFMatcher
    }
}
