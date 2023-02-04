import cv2
import numpy as np
 #画像の宣言
pic_name = "C:/sample3.jpg" #送られてきた画像に変更
pic_name_out = "red_out.jpg" 
gray_pic = "red_gray.jpg" 
    #ベース画像の読み込
img = cv2.imread(pic_name,cv2.IMREAD_COLOR)
    #画像のサイズ情報取得
height, width = img.shape[:2]
    #画像の色をHSV形式に変換
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
h = hsv[:, :, 0]
s = hsv[:, :, 1]
v = hsv[:, :, 2]
#ベース画像と同じ大きさの配列を作成
img=np.zeros((height,width,3),np.uint8)
#赤色を指定
img[(h <270) & (h > 240) & (s > 50) & (v > 50)] = 255 #屋外でのHSVのログから決めるしかなさそう
#赤の領域だけの画像を作成
cv2.imwrite(gray_pic,np.array(img))
#作成した画像を読み込み
img_gray = cv2.imread(gray_pic,cv2.IMREAD_GRAYSCALE)
#読み込んだ画像の重心、輪郭を取得
M = cv2.moments(img_gray, False)
contours, hierarchy= cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
x,y= int(M["m10"] / M["m00"]) , int(M["m01"]/M["m00"])
#ベース画像に重心、輪郭追加して保存
cv2.circle(img, (x,y), 20, 100, 2, 4)
cv2.drawContours(img, contours, -1, color=(0, 0, 0), thickness=5)
print(x,y)
cv2.imwrite(pic_name_out,np.array(img))
