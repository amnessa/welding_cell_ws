# Robot hattı: pozdan punta işaretine

Bu belge, FoundationPose sunucusundan **parça adı ve 4×4 poz** laptop'a geldikten sonra
olan her şeyi baştan sona anlatır:

1. Poz robot tabanına taşınır ve canlı bulutla iyileştirilir.
2. Parça takip edilir ve yerine konunca kaydedilir.
3. Robot parçalara yakından tekrar bakar (çok bakışlı iyileştirme).
4. CAD'lerden kaynak dikişleri ve puntalar hesaplanır.
5. Robot kalemle punta yerlerine dokunur.

Her adımda verinin **şekli, birimi ve hangi koordinat çerçevesinde olduğu** yazılıdır.

Sunucu tarafı (kamera, SAM2, nokta bulutu, PPF, FoundationPose) ayrı belgede:
[perception_aciklama.md](perception_aciklama.md). Bu belge onun bittiği yerden, Bölüm 11
"Laptop: cevabı almak ve yayınlamak"tan devam ediyor.

Bilinen yöntemler (ICP, ters kinematik, RRT-Connect, Gauss-Newton) anlatılmıyor;
yalnızca **ne için** ve **hangi ayarla** kullanıldıkları yazılı. Bu projeye özel olan
kısımlar ayrıntılı anlatılıyor: koordinat zinciri, kaydetme, çok bakışlı iyileştirme,
dikiş kuralı, punta kuralı, ölçüm zinciri.

İngilizce teknik referans: [README.md](../README.md), özellikle §14 (hata bütçesi),
§15 (çok bakışlı iyileştirme), §16 (derinlik kalibrasyonu). Deney günlüğü:
[thesis_notes.md](thesis_notes.md).

---

## İçindekiler

1. [Genel bakış](#1-genel-bakış)
2. [Koordinat zinciri: kameradan robot tabanına](#2-koordinat-zinciri-kameradan-robot-tabanına)
3. [Kalibrasyon: zincirin her halkası neyle ölçüldü?](#3-kalibrasyon-zincirin-her-halkası-neyle-ölçüldü)
4. [Canlı nokta bulutu](#4-canlı-nokta-bulutu)
5. [FoundationPose pozu + ICP ile ilk oturtma](#5-foundationpose-pozu--icp-ile-ilk-oturtma)
6. [Takip: parça elle taşınırken](#6-takip-parça-elle-taşınırken)
7. [Kaydetme: parça yerine konunca](#7-kaydetme-parça-yerine-konunca)
8. [Çok bakışlı yakın mesafe iyileştirme](#8-çok-bakışlı-yakın-mesafe-iyileştirme)
9. [Kaynak dikişi: CAD'lerden hesaplama](#9-kaynak-dikişi-cadlerden-hesaplama)
10. [Puntalar: nereye, kaç tane, hangi sırayla?](#10-puntalar-nereye-kaç-tane-hangi-sırayla)
11. [Erişilebilirlik: kalem her puntaya ulaşabilir mi?](#11-erişilebilirlik-kalem-her-puntaya-ulaşabilir-mi)
12. [Hareket ve işaretleme](#12-hareket-ve-işaretleme)
13. [Doğrulama: hata nasıl ölçülüyor?](#13-doğrulama-hata-nasıl-ölçülüyor)
14. [Her aşamada verinin özeti](#14-her-aşamada-verinin-özeti)
15. [Toplantıdan (1 Ekim) gelen konular ve durumları](#15-toplantıdan-1-ekim-gelen-konular-ve-durumları)

---

## 1. Genel bakış

```
 FoundationPose sunucusundan:  parça adı  +  T_kamera←parça (4×4, m)  +  maske
                                    │
┌─────────────────────── LAPTOP (ROS 2, icp_pose_refiner_node) ──────────────────────┐
│                                    ▼                                                │
│  [5] ilk oturtma     CAD'i T'ye koy, maskelenmiş canlı bulutla ICP ──▶ düzeltilmiş T │
│                                    │                                                │
│  [6] takip           FoundationPose, laptop GPU'sunda, her kare (25–29 Hz)          │
│                      (parça elle taşınırken poz izlenir)                            │
│                                    │                                                │
│  [7] kaydet          parça durunca: son pozların sağlam ortalaması → 5 canlı        │
│                      bulutta ICP → robot tabanında sabit CAD (SEPC) + assembly.json │
│                                    │   (ikinci parça için [5]–[7] tekrar)            │
│                                    ▼                                                │
│  [8] refine_pose     robot 4 yakın görüşe (0.40 m) gider, her birinde 16 kare        │
│                      → bütün parçalar birlikte iyileştirilir                         │
│                      → kameranın kendi konum hatası da tahmin edilip çıkarılır       │
│                      → parçalar arası aralık ISO 5817'ye göre kontrol edilir         │
│                                    │                                                │
│  [9] welding_points  CAD'ler pozlarında → yüz çiftleri → D4 kuralı                   │
│                      → kaynaklanabilir dikişler (çizgi, mm, robot tabanı)            │
│  [10] puntalar       tackrule-0.1: kalınlığa göre uzunluk, aralık, sıra              │
│                                    │                                                │
│  [11] erişilebilirlik  her punta için kalem açısı: çarpışmasız mı?                   │
│  [12] işaretleme     APS yol planı → yaklaş → kuvvetle durdurulan iniş → nokta/çizgi │
│                                    │                                                │
│  [13] doğrulama      kalem dokunuşları: işaret kökten kaç mm uzakta?                 │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

Özetle sunucu **"bu parça, kameraya göre şurada"** diyor. Laptop:

1. bunu robot tabanına çeviriyor;
2. canlı görüntüyle düzeltiyor;
3. parça yerine konunca sabitliyor;
4. yakından tekrar bakıp milimetreye indiriyor;
5. CAD'lerden **dikişi hesaplıyor** (dikiş görüntüde aranmıyor);
6. robotu oraya götürüyor.

**Şu anki sonuç (6 Ekim 2026):** FoundationPose takibi, kaydetmede ICP ve APS yol planıyla
iki plakalı T bağlantıda bütün puntalar köke **2.3–2.7 mm** yakınlıkta işaretlendi (ölçüm
hatası içinde olabilir). 24 Eylül'de bu hata ~8 mm, 2 Ekim'de (ICP takibiyle) ~2.5 mm idi.

---

## 2. Koordinat zinciri: kameradan robot tabanına

FoundationPose'un verdiği poz **kameraya göre**. Robot ise yalnızca kendi tabanına
(`base_link`) göre konum anlıyor. Arada üç dönüşüm var:

```
T_taban←parça  =  T_taban←bilek(q)  ·  T_bilek←kamera  ·  T_kamera←parça
                  ───────────────     ──────────────     ───────────────
                  ileri kinematik     el-göz             FoundationPose
                  (eklem açıları q)   kalibrasyonu       + ICP
                  her an değişir      sabit              her karede
```

- **`T_taban←bilek(q)`**: robotun eklem açılarından ileri kinematikle hesaplanıyor.
  Robot hareket ettikçe değişiyor.
- **`T_bilek←kamera`**: kamera robotun bileğine vidalı (**eye-in-hand**). Bu dönüşüm sabit
  ve bir kez kalibre ediliyor (`notebooks/T_tcp_to_cam.npy`).
- **`T_kamera←parça`**: görüntüden gelen poz.

Her 4×4 matris aynı biçimde: sol üst 3×3 dönme, sağ sütun öteleme (metre). Çarpım
soldan sağa okunuyor: parçadaki bir nokta önce kameraya, sonra bileğe, sonra tabana
taşınıyor.

**Kameranın robotta olmasının üç sonucu:**

1. **Kaydedilen her şey robot tabanında tutuluyor.** Kamera çerçevesi robotla birlikte
   hareket ediyor, taban sabit.
2. **Her bulut, çekildiği anın robot pozuyla eşleştiriliyor.** TF'e "şimdi" değil,
   bulutun zaman damgası soruluyor. Robot hareket halindeyken 50 ms'lik bir kayma bile
   birkaç milimetre demek.
3. **Bir maske yalnızca çekildiği kare için geçerli.** Robot hareket ederse aynı piksel
   başka bir yere bakar.

**Zincirin hatası her halkanın hatalarının toplamı.** Görüntüden gelen poz mükemmel olsa
bile:
- kamera bilekte 3 mm yanlış yerde varsayılırsa, parça da 3 mm yanlış yerde görünür;
- robot kinematiği 3 mm yanlışsa, kalem 3 mm yanlış yere gider.

Bu yüzden her halka ayrı ayrı ve **birbirinden bağımsız bir referansa** göre ölçüldü
(Bölüm 3).

---

## 3. Kalibrasyon: zincirin her halkası neyle ölçüldü?

Temel kural: **bir halkayı, ona bağlı olmayan bir şeyle ölç.** Kameranın hatasını kamerayla
ölçemezsin. Referans olarak robotun kendisi kullanıldı: kalemin ucu masaya ve parçalara
dokundurularak.

| sıra | halka | yöntem | sonuç |
|---|---|---|---|
| 0 | robot kinematiği | UR5e'nin fabrika kalibrasyonu (`ur5e_calibration.yaml`), sürücüde ve bütün araçlarda | nominal model ucu 2.4–4.2 mm kaydırıyordu, şimdi < 0.1 mm |
| 1 | kalem ucu (TCP) | el kumandasında 4 noktalı TCP; masaya 4 farklı bilek açısıyla dokunarak kontrol | yanal 0.2 mm |
| 2 | masa düzlemi | kalemle 7 noktaya dokunup düzlem oturtma (`table_touchoff.py`) | masa 1.57° eğik; düzlem referans oldu |
| 3 | kamera derinliği | RealSense **Tare** kalibrasyonu; gerçek mesafe robottan hesaplanıyor (`tare_distance.py`) | 0.3–0.65 m arasında 1.1 mm |
| 4 | el-göz dönmesi | kamera masayı 4 farklı bilek açısından görüyor; kameraya bağlı eğim masanınkinden ayrılıyor | ≈ 0.07° |
| 5 | el-göz ötelemesi | her `refine_pose` çalışmasında görüntülerden tahmin ediliyor (Bölüm 8.7) | yatay kısım düzeltiliyor; dikey kısımda 3–5 mm fazla düzeltme kaldı |

**Sıra önemli.** Bir adım, daha önceki düzeltilmemiş bir hatanın üstünde yapılırsa o
hatayı kendi sonucu sanıp öğrenir. Örneğin derinlik hatası düzeltilmeden yapılan el-göz
tahmini, derinlik hatasını kamera konumu hatası olarak kaydetti.

**Derinlik hatası menzilin karesiyle büyüyor** (Bölüm 8.1'in gerekçesi). Stereo kamerada
küçük bir eşleşme (disparity) kayması δ, derinlikte şu hatayı veriyor:

```
Δz = z² · δ / (f · B)          f: odak uzaklığı (piksel), B: iki kamera arası mesafe
```

Aynı masa noktasına üç yükseklikten bakıldı. Tare'den önce kameranın gördüğü masa ile
kalemle ölçülen masa arasındaki fark:

| uzaklık | 291 mm | 439 mm | 626 mm |
|---|---|---|---|
| Tare'den önce | +1.92 mm | +0.31 mm | −3.28 mm |
| Tare'den sonra | +0.29 mm | +1.06 mm | +1.43 mm |

Tare'den önceki değerlere `a + k·z²` biçiminde bir eğri birebir oturuyor; düz çizgi
oturmuyor. δ ≈ 0.6 piksel. Tarama konumundan (~0.6 m) bakınca bu hata her parçayı
~7 mm aşağıda gösteriyordu.

İlk Tare'de mesafe elle ölçüldü ve fazla düzeltti (%1.1 ölçek hatası). İkincisinde
mesafe robottan hesaplandı:

- kalibre edilmiş kinematik;
- el-göz dönüşümü;
- kalemle ölçülmüş masa düzlemi.

---

## 4. Canlı nokta bulutu

`depth_image_proc`, kameranın renk görüntüsüne hizalanmış derinliğini **düzenli (organized)
bir nokta bulutuna** çeviriyor. Geri izdüşüm formülü perception belgesinin Bölüm 7.1'indeki
formülün aynısı.

- **Şekil:** H×W×3, kameranın çözünürlüğünde (şu an 1280×720). Her piksel bir (x, y, z),
  metre, kamera çerçevesi.
- **Geçersiz pikseller** NaN. Silinmiyorlar, delik olarak kalıyorlar.
- **"Düzenli" olmasının faydası:** görüntüde komşu pikseller uzayda da komşu. Normal,
  komşu arama yapmadan bulunuyor: sağ–sol ve alt–üst farklarının vektörel çarpımı.
  Normaller kameraya doğru çevriliyor.
- **Seyreltme:** voksel ızgarası, her vokselden ortalama nokta.

Bu bulut sunucuya **gitmiyor.** Sunucuya giden yalnızca tek bir RGB + depth karesi.
Takip, kaydetme ve iyileştirme bu canlı bulutla laptop'ta yapılıyor.

---

## 5. FoundationPose pozu + ICP ile ilk oturtma

**Giriş:**
- sunucudan parça adı, `T_kamera←parça` ve maske (`detection_ism.npz`);
- canlı bulut.

**Çıktı:** düzeltilmiş `T_kamera←parça` ve takibin başlangıç pozu.

1. Adla aynı isimli CAD yükleniyor: `models/<ad>.ply`, mm → m.
2. CAD yüzeyinden noktalar örnekleniyor. Her noktaya üzerinde durduğu üçgenin **dışa
   bakan normali** veriliyor.
3. Canlı buluttan maskenin altındaki noktalar alınıyor.
4. FoundationPose pozundan başlayan noktadan-düzleme (point-to-plane) ICP çalıştırılıyor.
   Robust sürüm: Welsch ağırlıkları, Anderson hızlandırma. İyi bir başlangıç pozu ICP için
   şart; o pozu FoundationPose veriyor.

**Bize özel ek: normal kapısı (29 Eylül).** Kamera bir plakanın yalnızca bir yüzünü
görüyor. Ama CAD noktaları **her** yüzden örnekleniyor. Kapı olmadan arka yüzün noktaları
da en yakın sahne noktasını ön yüzde buluyor.

8 mm'lik ince bir plakada bu yanlış bir çözüm yaratıyordu: model kendi iki yüzünün
arasına "yaslanıyor", bir ucunda ön yüz, diğer ucunda arka yüz veriye oturuyordu. Sonuç
6.5° eğim ve kaynak kökünde 10 mm hata. ICP'nin kendi skoru ise iyi görünüyordu.

Çözüm: bir CAD noktası, yalnızca normali sahne noktasının normaliyle **60°'den az açı
yapıyorsa** eşleşiyor. Arka yüz ~180° ters baktığı için eşleşmeden çıkıyor. Aynı veride
eğim 0.3–0.8°'ye, kök hatası ±0.2 mm'ye indi.

Bu belgedeki bütün ICP çağrıları bu kapıyı kullanıyor.

---

## 6. Takip: parça elle taşınırken

**Senaryo:** parça masanın kenarında tanıtılıyor (sunucu adını ve pozunu buluyor). Sonra
elle alınıp yerine konuyor. Bu sırada sistem parçanın nerede olduğunu bilmeye devam
etmeli.

**Yöntem (6 Ekim'den beri varsayılan): FoundationPose ile takip** (`tracking_source: fp`).

- Kayıt (SAM2 + PPF + FoundationPose `register`) **masaüstünde** kalıyor: tam karede
  kayıt laptop'un 8 GB'lık GPU'suna sığmıyor.
- Takip **laptop'ta**, aynı Docker imajında çalışıyor (`fp_track_server.py`). Yalnızca
  iyileştirme ağı yükleniyor (~300 MB). `track_one` son pozdan başlayıp her karede
  pozu biraz düzeltiyor.
- `fp_tracker_node` her RGB-D kareyi yerel bağlantıdan (WebSocket) gönderiyor. Her karede
  en fazla 2 kare yolda, en yenisi kazanıyor.
- **Ölçülen:** 25–29 Hz, kareden poza 70–90 ms. Kaybolursa (uyum < 0.5, 3 kare) son pozdan
  çizilen maskeyle masaüstünde yeniden kaydediliyor (~4 s), tık gerekmiyor.
- **Robotun kendi hareketi** her karenin zaman damgasındaki TF ile çıkarılıyor: bileğe
  bağlı kamera hareket edince parça hareket etmiş görünmüyor.
- ICP düğümü her pozu o karenin zaman damgasıyla robot tabanına çeviriyor, parçayı
  RViz'de yeşil model bulutu olarak gösteriyor ve kaydetme için topluyor (Bölüm 7).

**Neden?** ICP CPU'da çalışıyor ve hareketli bir parçayı izlemek için yavaş. Literatürde de
ICP genellikle **duran** bir nesnenin bulutlarını eşlemek için kullanılıyor. ICP yalnızca
duran parçalarda kalıyor: kaydetmede (Bölüm 7) ve `refine_pose`'da (Bölüm 8). Kalman
füzyonu yok.

**Eski yöntem (`tracking_source: icp`): ICP ile takip.** Her karede maske yerine üç
geometrik süzgeç:

1. **Yönlü kutu:** CAD'in kutusu, kenar payıyla büyütülüp son poza konuyor.
2. **Masa çıkarma:** ölçülmüş masa düzleminin 2 mm üstüne kadar olanlar atılıyor.
3. **Kayıtlı parçaları çıkarma:** daha önce kaydedilmiş parçaların CAD bulutuna (SEPC)
   yakın noktalar atılıyor; ikinci parça birinciye yapışmıyor.

Kalan noktalarla ICP; bir adım 640×480 bulutta ~23 ms. Bu süzgeçler kaydetmedeki ICP'de
(Bölüm 7) aynen kullanılıyor.

---

## 7. Kaydetme: parça yerine konunca

**Sorun.** Duran bir parçada bile ICP takibinin pozu her karede biraz oynuyor. 25 Eylül'de,
parça ve robot hareketsizken ölçülen değerler:

| ölçü | değer |
|---|---|
| konum sapması (std) | 5 mm |
| konum aralığı (en büyük − en küçük) | 24 mm |
| en büyük dönme | 6° |

Kaydetme anındaki tek bir kare bu dağılımdan rastgele bir örnek demek. Toplantıda
sorulan "takipte hata birikir mi?" sorusunun pratik karşılığı bu.

**Çözüm** (`~/save_object`, `pose_stats.py`):

1. **Yalnızca son durma yerine ait kareler alınıyor.** En yeni pozdan geriye doğru
   yürünüyor. Bir kare, o ana kadarki karelerin medyanına 8 mm / 3° yakınsa durma yerine
   ait sayılıyor. Arka arkaya 3 uyuşmayan kare "parça taşındı" demek ve orada duruluyor.
   Böylece parça kaydetmeden hemen önce elle itildiyse, eski yeri ortalamaya karışmıyor.
2. **Sağlam ortalama:**
   - öteleme: eksen başına medyan;
   - dönme: dönme matrislerinin toplamının SVD ile en yakın dönmeye izdüşümü
     (chordal mean);
   - ICP'nin başka bir yerel minimuma atladığı kareler atılıyor.
3. **Sonuç robot tabanında sabitleniyor:**
   - `pose_static` (4×4, m, `base_link`);
   - dağılım (`pose_stats`) da yanına yazılıyor;
   - CAD noktaları bu pozda **SEPC**'ye (Static Environment Point Cloud) ekleniyor. SEPC,
     masaya konmuş her şeyin CAD modeli; Bölüm 6'da çıkarılan ve Bölüm 9'da dikişin
     hesaplandığı geometri bu.

Bunlar `assembly.json` dosyasına da yazılıyor: parça adı, poz, dağılım ve parçayı
kaydeden kameranın pozu (`T_static_camera`). Bu sonuncusu Bölüm 8.7'de lazım oluyor.

**4. FoundationPose takibinde: kaydetmeden önce ICP** (`fp_save_icp`, 6 Ekim).

- **Neden:** takip pozu "parça şu an nerede" sorusunun cevabı, kaynak için gereken
  "tam olarak nerede" değil. İlk denemede yalnızca takip pozuyla kaydedilen parçalar
  birbirinin **6–11 mm içine** girdi, `refine_pose` iki parçayı da reddetti ve izler
  12 mm erken düştü. Dokusuz 8 mm'lik plakada, ~0.6 m'den, FoundationPose'un render
  karşılaştırması takip için yeterli, kaynak için değil.
- **Ne yapılıyor:**
  1. ortalama takip pozundan başlayarak 5 yeni canlı bulutta ICP çalıştırılıyor
     (Bölüm 6'daki süzgeçler ve normal kapısıyla);
  2. sonuçlar robot tabanında ortalanıyor.
- **Ne zaman kullanılıyor:** yalnızca uyum ≥ 0.2 ve düzeltme ≤ 20 mm / 8° ise. Değilse
  takip pozu kaydediliyor ve cevap nedenini söylüyor.
- **Sonuç:**
  - render edilmiş bir plakada 4.7 mm / 1.5°'lik takip hatası, normal yönünde 0.01 mm'ye
    ve 0.4°'ye indi;
  - robotta: iki parça da `refine_pose`'da kabul edildi, aralık 0.3–2.9 mm, izler
    2.3–2.7 mm.
  - Kalan 1–2 mm plakanın kendi düzleminde; tek bakışın ölçemediği yön, `refine_pose`'un
    işi.

**Önemli:** kaydedilen şey kameradan gelen nokta bulutu değil, **CAD'in kendisi**. Gürültülü
sahne noktaları yerine kusursuz geometri. Bundan sonra gürültü yalnızca pozda var,
şekilde yok.

---

## 8. Çok bakışlı yakın mesafe iyileştirme

Servis: `ros2 service call /icp_pose_refiner/refine_pose std_srvs/srv/Trigger`. Kaydetme
ile dikiş hesabı arasında, tek komutla çalışıyor. Robot kendi kendine dört görüşe gidip
geliyor; sonuç RViz'de görünüyor ve hat oradan devam ediyor.

Kod: `multiview.py` (görüş seçimi), `multiview_capture.py` (çekim, ön işleme),
`multiview_refine.py` (iyileştirme), `multiview_service.py` (servis).

### 8.1 Neden?

Toplantıda söylendiği gibi **parçayı tanımak ile pozunu hassas bulmak iki ayrı iş.**

- Tanıma için uzaktan (tarama konumu, ~0.6 m) tek bir görüntü yetiyor.
- Hassas poz için iki sorun var:
  1. **Derinlik hatası menzilin karesiyle büyüyor** (Bölüm 3). 0.6 m'de 0.4 m'dekinin
     ~2.25 katı. Tare sonrası bile bir kalıntı var.
  2. **Tek bakış bazı yönleri hiç ölçemiyor.** Dik duran ince bir plakanın kendi
     düzlemindeki kayması ve dönmesi tek açıdan çok zayıf görünüyor. Gören yüzeyler bu
     harekete paralel.
- Ayrıca tek görüşteki **kamera kalibrasyon hatası** her kaydedilmiş poza aynen geçiyor.

Çözüm: parçalar kaydedildikten sonra robot kamerayı **0.40 m'ye** (D435i'nin minimum
mesafesi ~0.28 m) **dört farklı açıya** götürüyor. Bütün parçalar bütün görüşlere
**birlikte** oturtuluyor.

### 8.2 Görüşlerin seçimi

**Hedef:** kaydedilmiş parçaların ortak kutusunun merkezi. Tek nokta.

**Adaylar:**
- merkeze 0.40 m uzaktan bakan kamera pozları;
- yükseklik açısı 45° ve 60°;
- yatayda 30°'de bir;
- her biri için 4 farklı kamera dönmesi (roll).

**Neye göre puanlanıyor?** Önemli olan bütün parça değil, **dikiş bölgesi**: bir parçanın
yüzeyinde başka bir parçaya 1–30 mm uzaklıktaki noktalar. Bir aday, bu noktalardan kaçını
**iyi** görüyorsa o kadar puan alıyor. Bir noktanın iyi görülmesi için:

- kameraya uzaklığı > 0.28 m;
- görüş alanının içinde (69° × 42°);
- yüzeye bakış açısı 60°'den dik (daha yatık bakılan yüzeyin derinliği kötü);
- başka bir parçanın kutusu tarafından örtülmüyor.

**Seçim (açgözlü, greedy):** her adımda en çok **yeni** bilgi veren aday seçiliyor. Bir
noktanın 1., 2. ve 3. kez görülmesi 1, 0.5 ve 0.25 değerinde, yani aynı yeri tekrar tekrar
görmek az değerli. Seçilen yönler birbirinden en az 30° ayrı. 4 görüş seçiliyor.

**Fiziksel kontroller,** yalnızca seçilmek üzere olan adaylarda (pahalı oldukları için):

- ters kinematik, robotun hep aynı kol duruşunda (dirsek yukarı);
- tekillik (bilek ve dirsek): kol açılara aşırı duyarlı olduğu yerlere gitmesin;
- çarpışma: kol kapsülleri, takım zarfı, parçaların kutuları, masa; 20 mm payla.

Geçmeyen aday atlanıyor ve bir sonraki iyi aday deneniyor. Planlama birkaç saniye sürüyor.

### 8.3 Çekim

Her görüşte:

1. Robot gidiyor. Hareket kuvvet sensörüyle izleniyor; beklenmedik bir temasta duruyor.
2. Eklem hızları sıfırlanınca 0.5 s daha bekleniyor.
3. **16 yeni kare** alınıyor. Her piksel için 16 derinliğin **medyanı** alınıyor. Bu,
   piksel gürültüsünü bastırıyor ve tek karelik sıçramaları atıyor.
4. Kameranın pozu, karenin zaman damgasında TF'ten okunuyor (`T_taban←kamera`).

Dört görüş bitince robot eve dönüyor. Bütün çekim diske yazılıyor
(`multiview/<zaman>/`: her görüşün bulutu, kamera pozu, o anki montaj). Bundan sonraki her
şey bu kayıtlı veri üzerinde **hesap.** Aynı hesap robotsuz da tekrar çalıştırılabiliyor
(`multiview_refine_offline.py`); deneylerin tekrarlanabilmesi bu sayede.

### 8.4 Ön işleme (görüş başına)

1. Normaller (Bölüm 4), robot tabanına çevirme.
2. Kaydedilmiş parçaların kutularına 3 cm'den uzak noktalar atılıyor.
3. Masa düzleminin 2 mm üstüne kadar olanlar atılıyor.
4. 3 mm voksel ile seyreltme.

**Çıktı:** her görüş için N×3 nokta + N×3 normal, robot tabanında, ve o görüşün kamera
yönelimi.

### 8.5 Hangi nokta hangi parçanın?

İki parça birbirine dayalı. Köşeye yakın bir sahne noktası iki parçaya da ait olabilir.
Yanlış parçaya verilirse o parçayı yanlış yöne çeker.

- **Sahiplik:** her sahne noktası, kaydedilmiş yüzeyi ona en yakın olan parçaya veriliyor.
- **Ölü bant:** iki parçaya da 4 mm'den yakın noktalar **hiçbirine verilmiyor.** Köşedeki
  belirsiz bölge dışarıda kalıyor.
- Hiçbir parçaya 15 mm'den yakın olmayan noktalar da kullanılmıyor.

### 8.6 Parçalar sırayla: "resim çerçevesini iki kişi düzeltir gibi"

Bütün parçaları tek seferde çözmek yerine **sırayla** gidiliyor (blok koordinat inişi):

```
tur 1:  taban fit edilir (kulak sabit)  →  kulak fit edilir (taban sabit)
        sahiplik yeniden hesaplanır
tur 2:  taban  →  kulak
        (hiçbir parça 0.2 mm / 0.05°'den fazla oynamıyorsa erken durur)
```

Önce en çok noktası olan parça çözülüyor, genellikle taban. En iyi kısıtlı olan o ve
üstündekilere referans oluyor. Bizim hücrede hep iki parça olduğu için 2 tur.

**Tek bir parçanın çözümü.** Bilinmeyen, kaydedilmiş poza göre küçük bir düzeltme ξ: 3
dönme + 3 öteleme. Dönme, **parçanın kendi merkezi** etrafında yazılıyor. Robot tabanının
orijini etrafındaki 1°'lik dönme, parçayı 9 mm kaydırırdı; dönme ile öteleme birbirine
karışırdı. En aza indirilen toplam üç terimden oluşuyor:

```
  Σ  w · ρ( n·(T p − q) )²           1) veri: CAD noktası p, en yakın sahne noktası q,
                                        normali n; noktadan-düzleme uzaklık
                                        (normal kapısı + Welsch ağırlığı ρ)

+ ξᵀ Σ⁻¹ ξ                           2) önsel: kaydedilmiş pozdan uzaklaşmaya ceza
                                        (Σ: 3 mm, 1°)

+ w_o Σ max(0, −sd − 0.5 mm)²        3) iç içe geçme yasağı: komşu parçaya bakan
                                        yüzlerdeki CAD noktaları, komşunun kutusuna
                                        0.5 mm'den fazla giremez (sd: kutuya işaretli
                                        uzaklık, içeride negatif)
```

Terimlerin anlamı:

1. **Veri terimi:** CAD yüzeyi sahne noktalarına otursun.
   - Her görüşün **toplam ağırlığı eşit.** Yakın görüş çok nokta verdi diye diğerlerini
     bastırmıyor.
   - Welsch ağırlığı, uzak eşleşmelerin etkisini yumuşakça sıfıra indiriyor. Bant genişliği
     önce gevşek tutuluyor, sonra sensör gürültüsüne (1.5 mm) kadar daraltılıyor.
2. **Önsel:** parçanın kaydedilmiş pozuna bağlı yumuşak bir yay. Verinin ölçemediği bir
   yönde parça başıboş kaymasın, kaydedildiği yerde kalsın.
3. **İç içe geçme yasağı:** gerçek parçalar birbirinin içine giremez. Bu tek yönlü bir kural:
   - parçalar arasında boşluk olabilir ve ceza yok;
   - içeri girerse ceza var.

   Yalnızca düz yüzler için yazıldı. Silindir, boru gibi kavisli parçalarda eğri yüzeye
   uzaklık gerekiyor; o parçalar gelince eklenecek.

Çözüm Gauss-Newton ile, en fazla 30 adım.

**Hangi yönler gerçekten ölçüldü?** Çözümün bilgi matrisi `H = Jᵀ W J`. Dönmeler parçanın
yarıçapıyla çarpılıyor ki hepsi milimetre cinsinden olsun. H'nin her özdeğeri λ bir yönün
ne kadar ölçüldüğünü söylüyor:

```
o yöndeki belirsizlik:  σ_yön = σ_gürültü / √λ          (σ_gürültü = 1.5 mm)
σ_yön ≤ 0.5 mm  →  "ölçüldü"
σ_yön >  0.5 mm  →  "ölçülemedi"
```

Ölçülemeyen yönde düzeltme veriden değil önselden geliyor. Rapor bunu ayrıca yazıyor ve
kabul kuralı (8.8) bu yönlere ayrıca bakıyor.

Ölçüt **mutlak** olmak zorunda. İlk denenen göreli kural ("en güçlü yönün %5'inden zayıfsa
ölçülemedi") dik plakanın kendi düzlemindeki dönmesini "ölçülemedi" saydı. Oysa o yönü
binlerce nokta sabitliyordu; sadece başka bir yön çok daha güçlüydü.

### 8.7 Kameranın kendi hatası: görüşlerin anlaşmazlığından ölçmek

Bu bölüm çok bakışın en önemli kazancı.

**Fikir.** El-göz kalibrasyonunun öteleme kısmı d kadar yanlış olsun: kamera bileğe,
varsayılan yerden 4 mm farklı monte edilmiş. Bu hata **kameranın kendi çerçevesinde** sabit
bir vektör. Ama her görüşte kamera farklı yöne döndüğü için robot tabanında **farklı yönlere**
kayma yaratıyor: görüş v'nin bulutu `R_v · d` kadar kayıyor (`R_v`, o görüşteki kamera
yönelimi).

```
 görüş 1: bulut  R_1·d  kadar kaymış   ─┐
 görüş 2: bulut  R_2·d  kadar kaymış    ├─▶  aynı parça her görüşte biraz farklı yerde
 görüş 3: bulut  R_3·d  kadar kaymış    │    → bu anlaşmazlık d'yi ele veriyor
 görüş 4: bulut  R_4·d  kadar kaymış   ─┘
```

Tek görüşle d görünmez: parça sadece biraz kaymış görünür. Farklı yönlerden bakan görüşler
**birbiriyle anlaşamaz** ve anlaşmazlığın şekli d'yi verir.

**Hesap.** Her sahne noktasının artığına d terimi ekleniyor:

```
r = n · (T p − q + R_v d)
```

Bu, d'ye ve her parçanın düzeltmesine göre doğrusal. Bütün sahip olunmuş noktalar üzerinden
tek bir en küçük kareler problemi kuruluyor ve ikisi birlikte çözülüyor.

- d'nin kendi bilgisi, parçaların düzeltmeleri hesaba katıldıktan sonra kalan bilgi (Schur
  tümleyeni):

  ```
  S = N_dd − N_dx N_xx⁺ N_xd
  ```

- d'nin yalnızca σ ≤ 0.5 mm ile belirlenen yönleri kullanılıyor.

**Çevrim içi düzeltme.** d yeterince büyükse (> 0.5 mm):

1. her görüşten `R_v·d` çıkarılıyor;
2. her **kaydedilmiş pozdan** da `R_tarama·d` çıkarılıyor. Kaydedilmiş poz da aynı yanlış
   kamerayla, tarama konumundan bulunmuştu; Bölüm 7'deki `T_static_camera` burada lazım.
3. İyileştirme tekrar çalıştırılıyor. Düzeltilmiş görüşler hâlâ bir d gösteriyorsa en fazla
   4 kez tekrarlanıyor.

İkinci adım olmadan bir tuzak çıkıyordu. Görüşler düzeltildi ama kaydedilmiş poz
düzeltilmedi. Parçanın ölçülen yönleri yeni yere gitti, ölçülemeyen yönleri önsel yüzünden
eski (yanlış) yerde kaldı. Sonuç sahte bir 8 mm "aralık değişimi" ve iki parçanın da
reddedilmesi.

**Neden ortak çözüm tek başına yetmiyor?** Bütün parçaları bütün görüşlere birlikte oturtmak
d'yi kendiliğinden yok etmiyor. d'nin bütün görüşlerde ortak olan kısmı pozların içinde
kalıyor. Sentetik testte d, ancak açıkça tahmin edildiğinde 0.2 mm'ye kadar bulundu.

**Çalışmalar arası birikim.** Her çalışma d'yi ve 3×3 bilgi matrisini kaydediyor; kayıtlar
el-göz dosyasının özetiyle (sha1) eşleştiriliyor. Ayrı sahnelerden en az 3 çalışma 1 mm
içinde anlaşınca düzeltilmiş bir el-göz dosyası yazılıyor. Elle onaylanıyor, otomatik
değil. Aynı sahnenin tekrarları bağımsız kanıt sayılmıyor.

**Bilinen zayıflık.** 45° ve 60°'lik görüşlerin hepsi yukarıdan bakıyor. d'nin **dikey**
kısmı, görüşlerin en zayıf ayırdığı yön. Şu anki ölçümlerde dikey kısım 3–5 mm fazla
düzeltiliyor. Plan: yüksekliği kalemle ölçülmüş masaya bağlamak, görüşlerden yalnızca
yatay kısmı almak.

### 8.8 Kabul ya da red

Bir parçanın yeni pozu şu durumlarda **reddediliyor** ve kaydedilmiş pozu korunuyor:

| kural | sınır |
|---|---|
| düzeltme (kameranın payı çıkarıldıktan sonra) | > 10 mm veya > 4° |
| eşleşme oranı (fitness) | < 0.2 |
| komşu parçanın içine girme | > 1 mm |
| dokunan iki parça arasındaki göreli değişim, **ölçülemeyen yönlerde** | > 2 mm veya > 1° |

Reddedilen parça kaydedildiği yerde tutuluyor ve diğerleri ona karşı bir kez daha
iyileştiriliyor. **Montaj ya birlikte iyileştirilir ya hiç:** kabul edilen bir parçanın
tutulan bir parçayla aralığı çok değişecekse o da tutuluyor. Rapor reddedilen parçanın
denenen düzeltmesini de yazıyor.

### 8.9 Montaj aralığı (fit-up) kontrolü

Parçaları yerleştiren kişi onları birbirinden fazla uzak koyabilir. Sistem bunu fark edip
**uyarmalı.**

- Bir parçanın komşusuna oturan alt yüzleri ile komşunun karşı yüzü arasındaki aralık,
  temas boyunca ölçülüyor.
- Sınır: **ISO 5817:2023, Tablo 1, no. 617** (köşe kaynağında kök aralığı), weld_generator'ın
  `root_gap_limit` fonksiyonuyla, a = 0.7·t. 8 mm plaka için:

  | kalite seviyesi | B | C (varsayılan) | D |
  |---|---|---|---|
  | izin verilen aralık | 1.06 mm | 1.62 mm | 2.68 mm |

- Aşılırsa **uyarı** veriliyor, **red değil.** Aralık ölçüm hatası değil, gerçek bir montaj
  durumu; operatöre söylenmesi gereken bir şey.

İlk denemede parçaları birbirine çeken bir "oturma" terimi vardı. Sentetik testte gerçek bir
3 mm aralığı uyarmadan kapattı. Bu yüzden kapalı: aralığı **varsaymak** değil **ölçmek**
gerekiyor.

### 8.10 Çıktı

Kabul edilen pozlar `assembly.json` dosyasına ve SEPC'ye yazılıyor; eski poz yanında
saklanıyor (`pose_before_refine`). RViz'de:

- görüşlerin bulutları, görüş başına bir renkle;
- kamera okları;
- yeni montaj.

**Son çalıştırmadan örnek (2 Ekim):**
- iki parça da kabul edildi;
- kamera hatası d = (1.6, 3.6, −5.9) mm (kamera çerçevesi), tarama konumunda 7.1 mm'lik bir
  düzeltme demek;
- kulağın tabana oturan yüzünde 0.6–2.0 mm aralık ölçüldü ve C seviyesi (1.62 mm) için uyarı
  verildi;
- ardından işaretlenen bütün puntalar ~2.5 mm içindeydi.

---

## 9. Kaynak dikişi: CAD'lerden hesaplama

Servis: `~/welding_points`. Kod: `seam_from_registration.py`. Kural weld_generator
projesinden (`weldgen/accessibility.py`).

**Ana fikir.** Kaydedilen şey CAD'in kendisi (Bölüm 7). Dikiş görüntüde **aranmıyor**; iki
CAD'in yüzlerinden **hesaplanıyor.** Dikişin tek hatası pozların hatası.

1. **Her parça bir ilkel geometri olarak tanımlı.** `models/weldgen_objects.json`
   (`build_weldgen_registry.py`), her CAD için:
   - türünü (`slab` = plaka/kutu);
   - CAD ile ilkel arasındaki dönüşümü;
   - CAD'e karşı doğrulamasını

   tutuyor. Küçük çentikli/tırnaklı plakalar dış zarflarıyla kabul ediliyor (en fazla %10
   eksik).
2. **İlkeller `pose_static` pozlarına konuyor,** mm cinsinden, robot tabanında.
3. **Her yüz çifti bir aday.** Dikiş çizgisi iki yüz **düzleminin** kesişimi; yüzlerin
   üst üste geldiği kısımla sınırlı.
4. **D4 erişilebilirlik kuralı** her adayı yargılıyor: bir dikiş, iki **dış** yüzün
   kesişimi olmalı ve iki yüzün **açıortayı boş alana çıkabilmeli.** Yani torç oraya
   girebilmeli.

   Beş bağlantı türü (T, köşe, alın, bindirme, kenar) için tek kural. Sınıf yüzlerden
   çıkıyor:

   | sınıf | yüzler |
   |---|---|
   | `fillet` (köşe) | açılı iki yüz |
   | `butt` (alın) | aynı düzlemde iki yüz |
   | `lap_toe` (bindirme) | kenar × yüz |
   | `edge` (kenar) | kenar × kenar |

   Reddedilen adaylar da sebebiyle saklanıyor (`bisector_blocked`, `no_contact` …). "Dikiş
   değil" de kayıtlı bir karar; planlayıcı bunlardan da kaçınmalı.
5. **Poz toleransı.** weld_generator'ın kuralı parçaların tam temas ettiğini varsayıyor.
   Ölçülen pozlarda ise parçalar ya birkaç mm içe giriyor ya ayrık duruyor. Bu yüzden
   temas toleransı yerine **poz toleransı** (10 mm) kullanılıyor:
   - yüzler kesişim çizgisine 10 mm içinde ulaşmalı;
   - her yüz diğerinin düzleminin 10 mm ötesine uzanmalı.

     Bu ikinci şart, tolerans plaka kalınlığını aşınca bir plakanın kendi alt yüzünün dik
     plakayla eşleşmesini önlüyor.

   Dikiş çizgisi (iki düzlemin kesişimi), ince plakanın kendi düzlemindeki kaymasından
   **etkilenmiyor**. Bu kayma tek bakışın ölçemediği yöndü.
6. **Her dikişe aralık değeri yazılıyor** (`fitup_mm`): her parçanın yakın kenarının diğerinin
   yüz düzlemine işaretli uzaklığı. Aralık > 0, içe girme < 0.

**Çıktı** (`welding_seams.json`, robot tabanı, mm):
- dikiş çizgisi (polyline);
- sınıf;
- torcun yaklaşma ekseni;
- `weldable` (kaynaklanabilir mi) veya red sebebi;
- `fitup_mm`.

**Sınır.** Şimdilik yalnızca **düz plakalar.** Boru-plaka ve boru-boru (elips, eyer eğrileri)
weld_generator'da var, ama hücrede kayıt defterine `tube` / `swept_slab` girdileri ve
eğri yüzeyler için iç içe geçme kuralı (8.6) gerekiyor.

---

## 10. Puntalar: nereye, kaç tane, hangi sırayla?

Aynı servis, kaynaklanabilir dikişlere punta yerleştiriyor. Kural weld_generator'ın
**tackrule-0.1**'i (`weldgen/tacks.py`). `t`, iki parçadan **ince olanın** kalınlığı.

```
punta uzunluğu   = clip(4t, 10, 50) mm
en büyük aralık  = min(33t, 400) mm
en küçük aralık  = 10t mm
uç payı          = max(2t, punta uzunluğu)     (açık dikişte uçlara bu kadar yaklaşılmaz)

açık dikiş:  L_etkin = L − 2·uç payı
             L_etkin ≤ punta uzunluğu ise  → ortaya tek punta
             değilse  n = max(2, ⌈L_etkin / en büyük aralık⌉ + 1)
             puntalar [uç payı, L − uç payı] aralığına eşit dağıtılır
```

**Örnek: tezgâhtaki T bağlantı.** 8 mm plakalar, dikiş boyu 250 mm.

| büyüklük | hesap | sonuç |
|---|---|---|
| punta uzunluğu | 4·8 | 32 mm |
| uç payı | max(16, 32) | 32 mm |
| L_etkin | 250 − 64 | 186 mm |
| en büyük aralık | min(264, 400) | 264 mm |
| punta sayısı | max(2, ⌈186/264⌉ + 1) | 2 |
| konumlar | | 32 mm ve 218 mm |

T'nin iki tarafı da kaynaklanabildiği için toplam 4 punta.

**Sıra** (`order`):
- önce uçlar, sonra aralıklar ikiye bölünerek;
- bir bağlantının iki tarafı sırayla değişiyor. Isı bir tarafta birikmiyor, çarpılma
  dengeleniyor.

**Çıktı** (`welding_tacks.json`), her punta için:
- nokta ve segment (`p0_mm`, `p1_mm`);
- dikişteki sırası (`tack_no`);
- kaynak sırası (`order`);
- yaklaşma ekseni.

> Toplantıda "22t'de bir" denmişti; kodda kullanılan aralık tavanı **33t**. Kural
> sürümlü olduğu için (tackrule-0.1) değer değişirse sürüm artırılıp her şey yeniden
> hesaplanabilir. Hangi kaynağın 22t dediği kontrol edilmeli.

---

## 11. Erişilebilirlik: kalem her puntaya ulaşabilir mi?

Hareketten önce, masa başında: `tack_reachability.py`.

**Takım modeli** (`config/pen_tool.json`):
- kalem ucu;
- dokunma kuvveti 1.5 N;
- yaklaşma mesafesi 35 mm;
- tutucu, kalem, kamera kolu ve kamera gövdesinin çarpışma zarfları (CAD'den).

**Çarpışma modeli:**
- robot kolu: eklem çerçevelerine bağlı kapsüller;
- takım zarfı;
- kayıtlı parçalar: kutu;
- masa: ölçülmüş düzlem.

**Ne aranıyor?** Her dikiş için kalemin **eğim açısı × kendi ekseni etrafındaki dönmesi**
kombinasyonları deneniyor. Ters kinematik hep aynı kol duruşunda (dirsek yukarı, tarama
konumunun duruşu). Kombinasyon başına üç durumdaki en küçük boşluk ölçülüyor:

1. yaklaşma pozu;
2. punta pozu;
3. aradaki düz iniş.

En küçük boşluğu **en büyük** olan kombinasyon seçiliyor. Sonuç `tack_reach.json` dosyasına
yazılıyor; RViz'de her punta yeşil (ulaşılır) veya kırmızı.

---

## 12. Hareket ve işaretleme

Düğüm: `tack_marking_node.py`, servisler `~/plan`, `~/next`, `~/all`, `~/home`, `~/abort`.
Hareket kodu ortak bir yürütücüde (`motion.py`); `refine_pose` da robotu aynı yürütücüyle
hareket ettiriyor.

**Plan** (`~/plan`):
- **puntadan puntaya geçiş:**
  - önce düz eklem yolu deneniyor;
  - çarpışıyorsa **OMPL'in AnytimePathShortening'i (APS)** çalışıyor (6 Ekim'den beri):
    - 4 paralel RRT-Connect;
    - yollarının en iyi parçaları birleştiriliyor (hybridization) ve kısaltılıyor;
    - 1 saniyelik süre boyunca yol iyileşmeye devam ediyor.
  - Çarpışma modeli bunun için C++'a taşındı (`_transit_cpp`):
    - Python modeliyle aynı sonucu verdiği test ediliyor: 8 800 durumda birebir aynı;
    - ~590 kat hızlı.
  - Sonuç: ön–arka geçişte eklem yolu tek bir RRT-Connect'e göre **~6 kat kısa**.
  - APS çarpışma payını 2 mm geniş tutuyor; yol Python modeliyle yeniden denetleniyor.
    Olmazsa RRT-Connect yedek olarak devreye giriyor.
  - Köşeler spline ile yuvarlatılıyor ve TOTG ile zamanlanıyor.
  - Kalem parçalardan 20 mm uzak tutuluyor (başlangıç ve bitiş pozlarının izin verdiği
    kadar).
- **iniş:** kalem ekseni boyunca, kartezyen bir doğru üzerinde bir ters kinematik zinciri.
  Erişilebilirlik raporunun seçtiği kalem dönmesi (roll) o noktada çarpışıyorsa diğer
  dönmeler deneniyor; kalem yuvarlak, dönme serbest.
  - 6 Ekim'de çizginin başlangıcında, uç noktanın 4 mm ötesinde kamera gövdesi taban
    plakasına giriyordu; 330° yerine 0° ile çözüldü.
- **eve dönüş.**
- **kapılar:** başlangıç eklemleri çarpışmasız olmalı, kol aynı duruşta olmalı, planın ilk
  noktası mevcut eklemlere 20° içinde olmalı.

**Yürütme**, her punta için:

1. **Kuvvet sıfırı yeniden ölçülüyor.** Kuvvet sensörünün sıfırı kayıyor; açılıştan sonra
   robot dururken 7 N gösterdi. Serbest hareketlerden önce ve yaklaşma noktasında sıfır
   yeniden ölçülüyor.
2. **Geçiş** robotun hızı ölçeklenen yörünge denetleyicisiyle. 8 N'u aşan her kuvvet hareketi
   durduruyor.
3. **İniş iki hızlı:** önce 20 mm/s, son 6 mm 2 mm/s. Hareket, kalem ekseni boyunca kuvvet
   **1.5 N**'a ulaşınca iptal ediliyor. İptal anındaki eklemler temas noktası demek.

   Kalem yaylı değil. Yavaş son kısım olmadan 1.5 N'da algılanan temas, durma mesafesi
   yüzünden ~10 N'a çıkıyordu.
4. **İşaret:**
   - nokta modu: temasta kısa bekleme;
   - çizgi modu (`stroke_mode:=tack|seam`): punta segmenti ya da bütün dikiş boyunca 10 mm'lik
     parçalarla. Derinlik temas noktasından ölçülüyor ve parçalar arasında kalem ekseni
     boyunca ölçülen kuvvetle düzeltiliyor.
5. **Geri çekilme** kalem ekseni boyunca.

`tack_marks.json` her temas için şunları kaydediyor:
- uç konumu;
- kalem ekseni boyunca derinlik, yani beklenen yüzeye göre ne kadar erken/geç dokundu;
- çizgi kuvvetleri.

Bölüm 13'teki ölçümlerin ham verisi bu.

**Karar (2 Ekim):** dokunma **algılama** olarak kullanılmıyor. Yöntem yalnızca görüntüye
dayalı; parçalara dokunulamayan bir senaryo da geçerli kalmalı. Kalem dokunuşları yalnızca:
- kalibrasyonda (Bölüm 3);
- hatayı ölçmek için (Bölüm 13)

kullanılıyor. Bir puntanın yerini düzeltmek için asla kullanılmıyor.

---

## 13. Doğrulama: hata nasıl ölçülüyor?

Kalem izi, bütün zincirin kameradan bağımsız testi. Zincir:

```
derinlik → el-göz → kinematik → kayıt → dikiş → plan → kalem ucu
```

Üç araç var:

| araç | ne ölçüyor | neye karşı |
|---|---|---|
| `check_registration.py` | kayıtlı her yüzün canlı buluta göre ofseti ve yüz boyunca eğimi (kaçıklığı) | canlı bulut. Aynı kamerayı kullandığı için **kalibrasyon hatasını göremez**; yalnızca ICP'nin oturmasını test eder. |
| `touch_probe.py` | kalem ucu elle bir yüzeye dokundurulunca: masaya, en yakın kayıtlı yüze ve puntalara uzaklık | robotun kendisi, kameradan bağımsız |
| `tack_marks.json` | işaretleme sırasındaki temas derinlikleri | beklenen yüzey |

**8 mm'den 2.5 mm'ye** (ayrıntı: README §14, [thesis_notes.md](thesis_notes.md)):

| tarih | belirti | sebep | çözüm | sonra |
|---|---|---|---|---|
| 24 Eyl | dokunuşlar 19–29 mm erken | el-göz çözümü sessizce başarısız olmuştu | numpy Park–Martin ile yeniden çözüm; artık her çözüm kendi artığını yazdırıyor | derinlik doğru, yanal 6 cm |
| 24 Eyl | yanal 6 cm | kalibrasyon el kumandasının aktif TCP'sinde alınmış, ROS onu bileğe asıyor | TCP kalemle kalibre edildi, dönüşüm bileğe taşındı | yanal ~8 mm |
| 25 Eyl | izler 5–8 mm | duran parçada ICP pozu 5 mm std ile oynuyor | kaydetmede sağlam ortalama (Bölüm 7) | |
| 28 Eyl | hâlâ 5–8 mm | masa 1.57° eğik; düz zemin kesimi masayı parçanın yanında bırakıyordu | zemin kesimi ölçülmüş düzlemi izliyor | 3–5 mm |
| 28 Eyl | kalibrasyonlar birbirini tutmuyor | kameraya bağlı 0.36° eğim, kalibrasyonun kendi belirsizliği içinde | masaya göre eğim düzeltmesi | 3.7–4.3 mm |
| 29 Eyl | model hatası | TF nominal kinematiği, robot kalibre kinematiği kullanıyordu | kalibre kinematik her yerde | |
| 29 Eyl | izler 14–16 mm'ye kötüleşti | ICP ince plakanın iki yüzü arasına yaslandı | normal kapısı (Bölüm 5) | kök ±0.2 mm |
| 1 Eki | temas 8.5–9 mm erken | D435i derinliği menzilin karesiyle uzun | robottan hesaplanan mesafeyle Tare (Bölüm 3) | 1.1 mm (0.3–0.65 m) |
| 2 Eki | izler 3–7 mm, bir taraf erken bir taraf geç | el-göz ötelemesi, tek bakışın payı, plakanın düzlem içi dönmesi | `refine_pose` (Bölüm 8) | **~2.5 mm** |
| 6 Eki | takip FoundationPose'a geçince parçalar 6–11 mm iç içe, izler 12 mm erken | kaydedilen poz takip pozuydu, ICP pozu değil | kaydetmede 5 bulutta ICP (Bölüm 7) + APS yol planı + dönme yedeği | **2.3–2.7 mm** |

**Şu anki hata bütçesi:**

| halka | büyüklük |
|---|---|
| kinematik | < 0.1 mm |
| kalem ucu | yanal 0.2 mm, boy ~2 mm içinde |
| derinlik | 0.3–0.65 m arasında 1.1 mm, sabit ≈ +2 mm kalıntı |
| kamera eğimi | ≈ 0.07° |
| el-göz ötelemesi | `refine_pose` her çalışmada tahmin edip düzeltiyor. Düzeltme gerçek (görüşlerin masa yüksekliği farkı 6.9 → 3.7 mm), ama sahneyi kalemle ölçülmüş masanın ~1.7 mm altında bırakıyor. Sıradaki: masayı tahmine katmak |
| parça kaydı | yüzeylerde ~0.4 mm; tabanın kendi düzlemindeki kayması 1–3 mm (kökü oynatmıyor) |
| sabitlenmemiş parçalar | mıknatıslı; tarama, işaretleme ve dokunuş arasında ~1 mm oynayabiliyor |

---

## 14. Her aşamada verinin özeti

| # | nerede | veri | şekil | birim | çerçeve |
|---|---|---|---|---|---|
| 1 | sunucu → laptop | parça adı, poz, maske | ad + 4×4 + 720×1280 | m | kamera |
| 2 | laptop | canlı bulut | H×W×3 (düzenli) + normaller | m | kamera |
| 3 | laptop | CAD örneği | N×3 + N×3 dışa bakan normal | m | CAD'in kendi çerçevesi |
| 4 | `run_icp` | düzeltilmiş poz | 4×4 | m | kamera → TF ile taban |
| 5 | takip (FoundationPose) | poz, her karede | 4×4 | m | kamera → TF ile taban |
| 6 | `save_object` | `pose_static` + dağılım; SEPC | 4×4; N×3 | m | **taban** (`base_link`) |
| 7 | `refine_pose` çekimi | 4 görüş × (16 kare medyanı + kamera pozu) | görüş başına H×W×3 + 4×4 | m | kamera (+ taban←kamera) |
| 8 | ön işleme | görüş başına nokta + normal | N×3 + N×3, 3 mm voksel | m | taban |
| 9 | iyileştirme | parça başına yeni poz, kamera hatası d, bilgi matrisleri, aralıklar | 4×4; 3; 6×6; mm | m / mm | taban (d: kamera) |
| 10 | `welding_points` | dikişler | çizgi + sınıf + eksen + `fitup_mm` | **mm** | taban |
| 11 | puntalar | nokta, segment, sıra | punta başına | mm | taban |
| 12 | erişilebilirlik | eğim, dönme, boşluklar | punta başına | derece, mm | taban |
| 13 | plan | eklem yörüngeleri | eklem açıları × zaman | rad, s | eklem uzayı |
| 14 | işaretleme | temastaki uç, derinlik, kuvvetler | punta başına | m, N | taban |

**Birim farkına dikkat:** pozlar ROS'ta metre, dikiş ve punta dosyaları milimetre
(weld_generator'ın birimi).

---

## 15. Toplantıdan (1 Ekim) gelen konular ve durumları

| konu (toplantıda) | durum |
|---|---|
| "Kameraya göre poz var, robot tabanına alıyor musun?" | Evet: Bölüm 2'deki zincir. Her halka ayrı ölçüldü (Bölüm 3). |
| "Takipte hata birikebilir; yerine koyduktan sonra bir kez daha poz hesapla." | Yapıldı, iki adımda: kaydetmede sağlam ortalama (Bölüm 7) ve yakından tekrar bakış (Bölüm 8). |
| "Sensörün en iyi ölçtüğü mesafeyi bul, oradan bak." | Derinlik hatası menzilin karesiyle büyüyor (ölçüldü, Bölüm 3). Görüşler 0.40 m'den; 0.32–0.35 m denenecek (minimum 0.28 m). |
| "Tanıma ile hassas poz iki ayrı paket." | Hattın yapısı bu: uzaktan tanıma ve kayıt, sonra yakından `refine_pose`. |
| "FoundationPose varken neden ICP ile takip? Bir tabloya dök." | Yapıldı (6 Ekim): takip FoundationPose ile (25–29 Hz, laptop GPU'su), ICP duran parçalar için kalıyor (Bölüm 6, 7). Karşılaştırma tablosu (gürültü, gecikme, kayma; `pose_jitter_probe.py`) sırada ([todo.md](todo.md), "Benchmarks"). |
| "Parçalar arasında üretimden kaynaklı aralık varsa bulunabilir." | Yapıldı: aralık ölçülüyor ve ISO 5817 no. 617 sınırını aşınca uyarı veriliyor (Bölüm 8.9). |
| "Dokunarak mı, dokunmadan mı?" | Karar: dokunmadan, yalnızca görüntü. Dokunuşlar yalnızca kalibrasyon ve ölçüm için (Bölüm 12). |
| Hocanın dikiş fikri: iki parçanın yüzeyinden rastgele birer nokta, her adımda kendi komşuları içinde diğerine en yakın olana atlıyor; durdukları yer dikişe çok yakın; çok çiftle dikiş oturtuluyor; kenarların neden "çekici" (attractor) olduğunun ispatı | **Başlanmadı.** Hattın dışında ayrı bir iş olarak, önce adım adım görselleştirilerek (boşluk tuşuyla ilerleyen) denenecek. Şu anki yöntem (Bölüm 9) düzlemlerin kesişimi; yeni yöntem kavisli parçalar ve CAD'siz durum için aday. |
| Lazer işaretçi: "şuraya punta atacağım" diye önce göstermek | Açık. Kalemin yanına 3B baskılı bir tutucuyla, dijital çıkıştan açılıp kapanan bir lazer. Kalibrasyon, robot dikken XY'de daire çizdirerek. |
| Belirsiz PPF sonucunda başka açıdan bakmak (next-best-view) | Açık. Görüş seçimi (Bölüm 8.2) buna temel olabilir. |
| ODTÜ ağında Tailscale engelli; FoundationPose evdeki bilgisayarda | Takip artık laptop'ta yerel çalışıyor; ağdan yalnızca kayıt (parça başına bir kare) ve kayıp parçanın yeniden kaydı geçiyor. Kayıt için VPN. |

**Hâlâ açık olan teknik işler** ([todo.md](todo.md)):
- yüksekliği masaya bağlayıp dikeydeki 3–5 mm'lik fazla düzeltmeyi gidermek;
- parçaları sabitlemek (fikstür veya kelepçe);
- kavisli parçalar: kayıt girdileri, eğri yüzeyler için iç içe geçme kuralı, boru dikişleri;
- FoundationPose canlı takibi.
