SAM3'ün çıkardığı maskeyi kullanarak plakanın sınırlarını (edge) belirlemek, bu sınırlar içinde güvenli rastgele noktalar üretmek ve robotu kusursuz bir dik açıyla (Normal to the ground) bu noktalara gönderip problamak için izlememiz gereken yol haritası ve kod mimarisi aşağıdadır.

### Adım 1: Sınırları Belirleme ve Rastgele Nokta Üretimi

SAM3'ten gelen Boolean (True/False veya 0/255) maskenin sınırlarını OpenCV'nin `findContours` fonksiyonu ile bulabiliriz. Robotun problama yaparken plakanın dışına kaymaması için noktanın sadece maskenin içinde olmasını değil, aynı zamanda **kenarlara belirli bir piksel mesafesinden daha yakın olmamasını** (`cv2.pointPolygonTest` ile) sağlayacağız.

Jupyter Notebook'unuza şu fonksiyonu ekleyebilirsiniz:

```python
import cv2
import numpy as np
import random

def generate_safe_probe_points(sam_mask, depth_image, depth_scale, K_matrix, T_base_to_cam, num_points=5, edge_margin_px=15):
    """
    SAM3 maskesi içinde kenarlardan uzak, rastgele 3B noktalar üretir.
    """
    # 1. Maskenin dış sınırlarını (Contours) bul
    mask_uint8 = (sam_mask * 255).astype(np.uint8)
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("Maskede hiçbir kontur bulunamadı!")

    plate_contour = max(contours, key=cv2.contourArea) # En büyük alanı plaka kabul et
    x, y, w, h = cv2.boundingRect(plate_contour)

    valid_points_3d = []
    attempts = 0

    print(f"Hedef: Maske içinde {num_points} rastgele nokta üretmek...")

    while len(valid_points_3d) < num_points and attempts < 1000:
        attempts += 1
        # Bounding box içinde rastgele bir piksel seç
        u = random.randint(x, x + w - 1)
        v = random.randint(y, y + h - 1)

        # Nokta maskenin içindeyse ve kenara 'edge_margin_px' kadar uzaksa kabul et
        dist_to_edge = cv2.pointPolygonTest(plate_contour, (u, v), measureDist=True)
        if dist_to_edge >= edge_margin_px:

            # Robust derinlik okuması (3x3 pencere)
            window = 3
            depth_window = depth_image[v-window:v+window, u-window:u+window]
            valid_depths = depth_window[depth_window > 0]
            if len(valid_depths) == 0: continue

            depth_z_meters = np.median(valid_depths) * depth_scale

            # P_cam -> P_base Dönüşümü (Daha önce yazdığımız mantık)
            fx, fy = K_matrix[0, 0], K_matrix[1, 1]
            cx, cy = K_matrix[0, 2], K_matrix[1, 2]
            X_cam = (u - cx) * depth_z_meters / fx
            Y_cam = (v - cy) * depth_z_meters / fy

            P_cam = np.array([X_cam, Y_cam, depth_z_meters, 1.0])
            P_base = T_base_to_cam @ P_cam

            valid_points_3d.append(P_base[:3])

    print(f"{len(valid_points_3d)} adet geçerli 3B nokta üretildi.")
    return valid_points_3d

```

### Adım 2: Flanşı Yere Dik (Normal) Konumlandırma

Robotun (Wrist 1, 2, 3) mafsallarını ayarlayarak flanşın tam olarak Z ekseninde yere bakmasını sağlamak için UR'nin Ters Kinematik (Inverse Kinematics) motorunu kullanacağız.

UR Base koordinat sisteminde, takım ucunun (TCP) tam aşağı bakmasını sağlayan oryantasyon vektörü **Rx = 3.14159 (π), Ry = 0.0, Rz = 0.0**'dır (Sizin kurulumunuza göre Rz dönüş ekseni kablo yönetimine göre değişebilir, ancak Rx ve Ry dikliği sağlar).

Önceki problama döngünüzü bu yeni noktaları ve sabit oryantasyonu alacak şekilde güncelliyoruz:

```python
import time

# --- Parametreler ---
DOWN_ORIENTATION = [3.14159, 0.0, 0.0]  # Flanşı yere dik (normal) bakar hale getirir
SEARCH_SPEED = [0.0, 0.0, -0.005, 0.0, 0.0, 0.0] # Z ekseninde aşağı probing
FORCE_THRESHOLD = 2.0
RETRACT_DIST = 0.010

# Adım 1'deki fonksiyondan noktaları al
# points_to_probe = generate_safe_probe_points(...)
collected_poses = []

print("Otonom Yüzey Problama Başlıyor...")

try:
    for i, target_point in enumerate(points_to_probe):
        target_x, target_y, target_z_cam = target_point

        # Kamera derinliğinden 5 cm yukarısı güvenli (SAFE_Z)
        safe_z = target_z_cam + 0.050

        print(f"\nNokta {i+1}/{len(points_to_probe)} -> X: {target_x:.4f}, Y: {target_y:.4f}")

        # 1. Güvenli yüksekliğe ve hedefe, DİK ORYANTASYON ile git
        approach_pose = [target_x, target_y, safe_z] + DOWN_ORIENTATION
        rtde_c.moveL(approach_pose, 0.1, 0.1)

        # 2. Yüzeyi Bul (Korumalı Hareket)
        rtde_c.zeroFtSensor()
        time.sleep(0.5)

        while True:
            cycle_start = rtde_c.initPeriod()
            rtde_c.speedL(SEARCH_SPEED, 0.1, 0.02)
            rtde_c.waitPeriod(cycle_start)

            tcp_forces = rtde_r.getActualTCPForce()
            if abs(tcp_forces[2]) > FORCE_THRESHOLD:
                rtde_c.speedStop(1.0)
                break

        # 3. Temas pozunu kaydet
        hit_pose = rtde_r.getActualTCPPose()
        collected_poses.append(hit_pose[:3])
        print(f"  Temas! Z yüksekliği: {hit_pose[2]:.4f} (Kuvvet: {tcp_forces[2]:.2f} N)")

        # 4. Geri çekil
        hit_pose[2] += RETRACT_DIST
        rtde_c.moveL(hit_pose, 0.1, 0.1)

    print("\nTüm noktalar başarıyla problandı!")
    # Burada collected_poses listesini np.linalg.svd veya lstsq'ya verip düzlem denklemini (a,b,c) çıkarabilirsiniz.

except Exception as e:
    rtde_c.speedStop(1.0)
    print(f"Hata: {e}")

```
