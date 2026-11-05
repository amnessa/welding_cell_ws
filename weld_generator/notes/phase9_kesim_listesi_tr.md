# Faz 9 gerçek test seti: kesim listesi (açıklamalı)

*2026-11-05. `phase9_real_set_plan.md` içindeki kesim listesinin Türkçe, terimleri açıklanmış
hali. Atölyede yanınızda olsun diye yazıldı. Her şey elle, el aletleriyle kesilecek (laboratuvarın
CNC router'ı çelik kesmeye izin vermiyor).*

**En önemli kural:** Testin "doğru cevabı" kesimden sonra yaptığımız **ölçümlerden** kurulur,
nominal ölçülerden değil. Bu yüzden boyun 1–2 mm farklı çıkması sorun değil. Ama **dikiş
kenarları** (parçanın diğer parçaya değip kaynak dikişinin oluşacağı kenar) **düz ve dik**
olmalı: boru uçları masada sallanmadan durmalı, sac kenarları çelik cetvele boşluksuz oturmalı.

## Terimler

| plandaki terim | Türkçesi | ne demek |
|---|---|---|
| stub | kısa boru parçası | İki ucu da **dik** (boru eksenine 90°) kesilmiş düz bir boru parçası. "C101 stub 120" = Ø101.6 borudan 120 mm boyunda, iki ucu düz kesilmiş parça. Evet, sadece 120 mm'lik düz bir kesit. |
| square cut | dik (düz) kesim | Kesim yüzeyi boru eksenine 90°. |
| mitre | açılı kesim (atölyede "gönye kesim" de denir) | Kesim yüzeyi boru eksenine göre **eğik**; kesim borunun tüm et kalınlığından geçer ve kesit bir elips olur. Boru açılı ucunun üstünde durunca yana yatık durur. |
| bevel | kaynak ağzı / pah | Kenarın **et kalınlığı boyunca** açılı taşlanması (kaynak ağzı açmak). Bu sette **hiçbir yerde yok**. Çapak alırken dikiş kenarlarına pah kırmayın. |
| fishmouth | balık ağzı (semer) kesim | Branşman borusunun ucunu, ana borunun yüzeyine oturacak şekilde eğrisel kesmek. |
| mitred fishmouth | açılı balık ağzı | Balık ağzı, ama branşman ana boruya dik değil, ana borunun ekseni boyunca yatık oturur (Y şeklinde). Bir tarafı diğerinden uzun olur. |
| run | ana boru | Yatay yatan büyük boru (R101); branşmanlar üstüne oturur. |
| branch | branşman (dal borusu) | Ana borunun üstüne balık ağzı ucuyla oturan küçük boru (S42, S60, S76). |
| ring | halka | Borudan dik kesilmiş kısa bir dilim (burada 70 mm). |
| arc (170°, 150°) | yay parçası | Halkayı **boyuna** (boru eksenine paralel) keserek çıkan kavisli şerit. 170° = tam çemberin 360°'sinin 170°'si, yani borunun yarısından biraz az. |
| RHS | kutu profil | Dikdörtgen/kare kesitli içi boş profil (60×60, 80×80, 50×100). |
| kerf | kesim payı | Diskin/bıçağın kalınlığı kadar kaybolan malzeme (1 mm diskle ~1–1.5 mm). |
| mid-wall | et ortası | Et kalınlığının ortası; yay boyları buradan ölçülür. |

## Önerilen aletler

- **Avuç taşlama (spiral), 1 mm inox kesme diski** (115 veya 125 mm). Diskleri **sadece
  paslanmazda** kullanın: karbon çelikte kullanılmış disk paslanmaza demir bulaştırır ve pas
  lekesi yapar.
- **Dekupaj testere**, paslanmaz için bimetal bıçak: yay çiftleri (R200, R250, R300) için.
- **Varsa metal kesme makinesi (daire veya şerit testere):** boruların ve kutu profillerin dik
  kesimleri için, elle taşlamadan çok daha düz ve dik keser.
- Eğeler (yassı, yarım yuvarlak), çapak için flap disk (zımpara diski).
- Ölçme: çelik cetvel, gönye (90° açı cetveli), kumpas, sentil (feeler gauge), metre, çizici
  veya ince markör.
- Mengene ve işkenceler.
- Koruma: yüz siperi, eldiven, kulaklık. Kıvılcımları robottan ve kameradan uzak tutun.
- **Paslanmazı yavaş, bastırmadan kesin.** Isınırsa sertleşir ve renk değiştirir.

## 1. Borular (304 paslanmaz, 2 mm et, her biri 1 m)

| boru | parça | boy (mm) | nasıl kesilir | ne için |
|---|---|---|---|---|
| Ø101.6 | **C101** | 120 | iki uç dik | plaka (B2) üstünde dik duran boru: dairesel dikiş |
| | **E101** | uzun kenar ~140, kısa kenar ~113 | önce ~140 dik kes, sonra bir ucu şablonla **15°** açılı kes | açılı ucu plakada, boru 15° yatık: eliptik dikiş |
| | **R101** | 250 | iki uç dik | ana boru: yatay yatar, S42/S60/S76 üstüne oturur |
| | **2 halka** | 70 | iki uç dik | 1. halkadan 170° ve 150° iki yay çıkar (aşağıda); 2. halka yedek |
| Ø76.1 | **C76** | 120 | iki uç dik | dik duran boru |
| | **E76** | uzun ~150, kısa ~122 | bir ucu **20°** açılı | yatık boru |
| | **halka** | 70 | iki uç dik | bir adet 170° yay (gerisi yedek) |
| | **S76** | 120 | bir ucu balık ağzı (Ø101.6'ya), diğeri dik | R101 üstünde branşman |
| Ø60.3 | **C60** | 120 | iki uç dik | dik duran boru |
| | **E60** | uzun ~160, kısa ~125 | bir ucu **30°** açılı | yatık boru |
| | **S60** | ~140 | bir ucu **açılı balık ağzı** (Ø101.6'ya, **20°** yatık); uzun kenarı kısa kenarından 22 mm uzun | yatık branşman |
| Ø42.4 | **C42** | 120 | iki uç dik | dik duran boru |
| | **S42** | ~130 | bir ucu **açılı balık ağzı** (Ø101.6'ya, **15°** yatık); uzun kenarı kısa kenarından 11 mm uzun | yatık branşman |

Kalan boylar (kesim payları dahil, yaklaşık): Ø101.6 ~300, Ø76.1 ~520, Ø60.3 ~530, Ø42.4 ~690
mm. Bunlar yedek; bir kesim bozulursa tekrar kesilir.

**Açılı uçlar ve balık ağızları** (açılı balık ağızları dahil): bir script her biri için
**1:1 sarma şablon** basacak.
Kağıdı borunun etrafına sarın, çizgiyi çizin, taşlamayla çizginin biraz dışından kesin, sonra
çizgiye kadar eğeleyin. Balık ağzını R101'in üstünde deneyerek eğeleyin. Aralığı (gap) sonra
şimlerle ayarlıyoruz, tam sıfır olması gerekmiyor.

### 170° ve 150° yaylar nasıl çıkar?

Halkanın yüksekliği (70 mm), yayın plaka üstünde duracağı yükseklik olur. Yay, halkanın **dik
kesilmiş dairesel kenarı** üstünde B2 plakasına oturur; dikiş, plaka üstünde bir yay çizer. Bu
yüzden halkanın iki dik kesimi düzgün olmalı, boyuna kesimler ise sadece yayın uçlarıdır.

**Ø101.6 halka** (dış çevresi 319 mm):
1. Halkanın dışına bir kağıt şerit sarın ve şu noktaları işaretleyin: **0**, **151**, **153**
   ve **286** mm.
2. Bu noktalardan boru eksenine **paralel** çizgiler çizin. Halkayı düz masaya yatırın ve
   gönyeyi ya da bir köşebent parçasını cetvel gibi kullanın.
3. Üç boyuna kesim yapın:
   - 0–151 arası: **170° yay** (dikiş uzunluğu, et ortasında, ~148 mm);
   - 153–286 arası: **150° yay** (dikiş uzunluğu ~130 mm);
   - 151–153 arası kesim payı; 286–319 arası ~33 mm'lik şerit fire.

**Ø76.1 halka** (dış çevresi 239 mm): 0 ve **113** mm'yi işaretleyin, iki boyuna kesimle **170°
yayı** (dikiş ~110 mm) çıkarın. Kalan ~190°'lik parça yedek.

## 2. Kutu profiller (RHS)

| profil | parça | boy (mm) | kesim |
|---|---|---|---|
| 60×60 | **RR60** | 100 | iki uç dik |
| | **RM60** | uzun ~120, kısa ~98 | bir ucu **20°** açılı |
| | **RF60** | 120 | bir ucu balık ağzı (Ø101.6'ya, dik) |
| | **RF60T** | uzun kenar ~140 | bir ucu **açılı balık ağzı** (Ø101.6'ya, ana boru boyunca **20°** yatık) |
| 80×80 | **RR80** | 120 | iki uç dik |
| | **RM80** | uzun ~130, kısa ~109 | bir ucu **15°** açılı |
| 50×100 | **RR50** | 120 | iki uç dik |
| | **RM50** | uzun ~130, kısa ~101 | bir ucu **30°** açılı, 100 mm'lik yüzlerden birine doğru yatar |
| | **RF50** | 120 | bir ucu balık ağzı (Ø101.6'ya, dik); 50 mm'lik kenar ana borunun eksenine dik |

Kalan boylar (yaklaşık): 60×60 ~500, 80×80 ~740, 50×100 ~620 mm.

- **RR** parçaları B2 plakası üstünde dik durur. Dikiş, profilin plakaya oturan ucunun
  etrafında oluşur; o uç düz olmalı, masada sallanmamalı.
- **RM** parçaları (açılı kesim) B2 üstünde yatık durur. Kesim, boru mitresinden kolaydır: iki
  yüzde kesim çizgisi açılı, diğer iki yüzde düz ama farklı yüksekliklerde. Şablon üstünde
  gösterilecek.
- **RF** parçaları (balık ağzı) R101 ana borusunun üstüne oturur. Ana borunun eksenine dik
  duran iki duvar yay şeklinde (ana borunun yarıçapı, 50.8 mm) kesilir; diğer iki duvar düz
  kesilir ama daha aşağıda biter. RF60T aynı şeyin 20° yatık hali. Bunlar da şablonla kesilir.
- **RM ve RF parçaları weldgen'de henüz yok** (sonra eklenecek). Şimdi kesiliyorlar; veri
  setine sonra girecekler.

## 3. 3 mm paslanmaz sac

| parça | ne | not |
|---|---|---|
| **B1** 300×240 | taban plakası (T dikiş ve bindirme testleri) | dikdörtgen |
| **B2** 200×200 | taban plakası (boru, profil ve yay testleri) | dikdörtgen |
| **P1, P2, P3** | T dikişi için plakaya dik duran 3 çokgen plaka | şekiller script'ten |
| **A, B, C çiftleri** | her çiftte birbirine uyan bir dikiş kenarı: köşe, alın ve bindirme testleri | şekiller script'ten |
| **R200, R250, R300 çiftleri** | yay kenarlı alın birleştirme: yarıçap 200, 250, 300 mm | her çift **tek plakadan, tek yay çizgisi boyunca** kesilir |

- **Çokgen plakaların ölçüleri henüz yok.** Generator'ın kendi şekillerinden üretilecek (plan
  maddesi `export_templates.py`). A4'e bölünmüş **1:1 şablonlar** basılacak: dikiş kenarları
  kalın çizgiyle ve etiketli, kontrol için 100 mm'lik bir ölçek çizgisi var. Baskıdan sonra
  ölçek çizgisini cetvelle kontrol edin, sonra şablonu saca yapıştırın.
- Tüm kenarlar düz çizgi, iç köşe yok. Kesimi çizginin biraz dışından yapın, çizgiye kadar
  eğeleyin. **Dikiş kenarını** çelik cetvelle düzlüğe, gönyeyle sacın yüzüne dikliğe kontrol
  edin.
- **Yay çiftleri:** tek plakayı yay çizgisi boyunca kesince çıkan iki parçanın kenarları
  birbirine tam uyar. Aradaki **kesim payı (~1–1.5 mm) kök aralığı olur**, ki bu tam istenen
  aralıktadır (0.3–2.0 mm). Dekupaj testere eğriyi en iyi izler; taşlamayla kısa geçişlerle de
  olur.

## 4. 1–2 mm karbon çelik sac

| parça | ne | not |
|---|---|---|
| **D, E, F çiftleri** | kenar birleştirme (edge joint) | şekiller script'ten; hem 1 hem 2 mm varsa bir çift farklı kalınlıkta |

- Sac makası ince sacın kenarını büker. Dikiş kenarlarını taşlamayla kesin, ya da makasla
  çizginin dışından kesip eğeleyin ve sonra düzeltin.
- Masada 0.5 mm'den fazla sallanan sac düz değildir: düzeltin veya yeniden kesin.
- Çabuk paslanır: kuru tutun, çekimden önce silin.

## Kesimden sonra ölçülecekler

- **Borular:** uç yüksekliği 4 noktada, dış çap 3 farklı açıda, et kalınlığı.
- **Kutu profiller:** dış ölçüler ve köşe yarıçapı.
- **Saclar:** her kenar, bir köşegen, kalınlık 4 noktada.
- **Yay çiftleri:** iki parça yan yana dururken aralık, sentille 3 noktada.

Ölçümler bir tabloya yazılır. Script parçaları bu ölçülerden üretir.
