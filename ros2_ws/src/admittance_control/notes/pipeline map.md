# Pipeline haritası: parçayı koymaktan punta işaretine

Her balon bir işlem, her ok o işlemin bir sonrakine taşıdığı veri.

![Pipeline haritası](pipeline_map.png)

*(Görüntünün kaynağı `pipeline_map.dot`. Değiştirince
`dot -Tpng -Gdpi=110 pipeline_map.dot -o pipeline_map.png` ile yeniden üretilir.)*

| balon | ne yapıyor | aldığı | verdiği |
|---|---|---|---|
| **CAD kütüphanesi** | Her parçanın üçgen ağı. Bir kez hazırlanıyor: yüzeyden örneklenen model bulutu (PPF için) ve parçanın kaynak ilkeli (plaka, dikiş hesabı için). | PLY mesh (mm) | model bulutları ~600×6 (CAD çerçevesi), CAD mesh, kaynak ilkeli |
| **Kalibrasyon zinciri** | Kameranın gördüğü noktayı robotun anlayacağı yere taşıyan dönüşümleri ölçüyor. Sırasıyla: robot kinematiği, kalem ucu, masa düzlemi, kamera derinliği (Tare), el-göz dönmesi. Referans robotun kendisi, yani kalem ucunun dokunuşları. | kalem dokunuşları, robot eklem açıları | kinematik, el-göz dönüşümü, masa düzlemi, kalem ucu |
| **Kamera** | Robotun bileğindeki D435i. Derinlik, renk görüntüsünün piksellerine hizalı. | sahne | RGB 720×1280×3, derinlik 720×1280 (mm), K 3×3 |
| **Operatör** | Parçanın üstüne tıklayarak hangi nesnenin kastedildiğini söylüyor. | – | tık (u, v) |
| **SAM2: hangi pikseller?** | Tıkı içeren sürekli bölgenin maskesini çıkarıyor. Yalnızca RGB kullanıyor; negatif tık taşan yeri oyuyor. | RGB + tık | maske 720×1280 (0/1) |
| **Maske → nokta bulutu** | Maske ⊙ derinlik; her pikseli K ile 3B'ye geri izdüşürüyor (`X = (u−cx)·Z/fx`, `Y = (v−cy)·Z/fy`). Kenarı aşındırıyor, normalleri hesaplıyor, seyreltiyor. | maske, derinlik, K | sahne bulutu N×6, N ≤ 2000: konum + normal, m, kamera çerçevesi |
| **PPF: hangi parça?** | İki nokta ve normalleri 4 sayı veriyor (1 uzaklık + 3 açı); bu sayılar poza bağlı değil. Sahnedeki çiftler her CAD'in tablosunda aranıp oy topluyor. En çok oylu 12 poz ICP ile düzeltiliyor ve kapsama × açıklama ile puanlanıyor. En yüksek skor kazanıyor. | sahne bulutu, model bulutları | parça adı, skor tablosu, margin |
| **FoundationPose: nerede?** | ~252 yönelim hipotezi üretiyor. Her birinde CAD'i render edip gerçek görüntüyle karşılaştırıyor ve düzeltiyor. En iyi puanlıyı seçiyor. | RGB, derinlik (m), K, maske, seçilen CAD | T_kamera←parça 4×4 (m) |
| **Robot tabanına çevir** | `T_taban←parça = FK(q) · T_bilek←kamera · T_kamera←parça`. Kamera robotla hareket ettiği için her şey robot tabanında tutuluyor. | poz, eklem açıları, el-göz | T_taban←parça 4×4 (m) |
| **Canlı nokta bulutu** | Her kamera karesinden düzenli bir bulut (H×W×3). Normaller piksel komşuluğundan. İlk oturtma, takip ve yakın bakış bu bulutu kullanıyor. | derinlik, K | bulut H×W×3 (m, kamera çerçevesi) |
| **İlk oturtma** | FoundationPose pozundan başlayan ICP. Normal kapısı: CAD noktası, normali 60°'den farklı bakan sahne noktasıyla eşleşmiyor. Böylece ince plakanın arka yüzü ön yüze yapışmıyor. | başlangıç pozu, maskenin altındaki canlı noktalar, CAD | düzeltilmiş poz |
| **Takip** | Parça elle yerine taşınırken her karede poz. CAD kutusuyla kırpıyor, masayı (kalemle ölçülmüş düzlem) ve daha önce kaydedilmiş parçaları atıyor, kalanla ICP yapıyor. | önceki poz, canlı bulut | poz akışı |
| **Kaydet** | Parça durunca son durma yerine ait pozların sağlam ortalamasını alıyor: medyan konum, ortalama dönme, sıçrayan kareler atılıyor. CAD'i bu pozda robot tabanına sabitliyor. | poz akışı | pose_static 4×4 (m, robot tabanı) + CAD; ikinci parçanın takibine "çıkarılacak" olarak |
| **Yakından tekrar bak** | Dikiş bölgesini en iyi gören 4 görüşü seçiyor (0.40 m, 45°/60°). Her görüşte 16 karenin medyanını alıyor. Parçaları sırayla bütün görüşlere oturtuyor; iç içe geçme yasak. Görüşlerin anlaşmazlığından kameranın kendi konum hatasını bulup çıkarıyor. Parçalar arası aralığı ISO 5817'ye göre kontrol edip aşılırsa uyarıyor. | kaydedilmiş pozlar ve CAD'ler, 4 görüşün bulutları ve çekim anındaki kamera pozları | iyileştirilmiş pozlar, aralık uyarıları |
| **Dikişler** | Her parçanın kaynak ilkelini pozuna koyuyor; yüz çiftlerinin düzlem kesişimini hesaplıyor. D4 kuralı: iki dış yüz olmalı ve açıortayları boşluğa çıkabilmeli, yani torç girebilmeli. Poz toleransı 10 mm. Dikiş görüntüde aranmıyor, CAD'lerden hesaplanıyor. | iyileştirilmiş pozlar, kaynak ilkelleri | dikiş çizgileri (mm, robot tabanı), sınıf, torç ekseni, aralık |
| **Puntalar** | Kalınlığa göre kural (t = ince parça): uzunluk 4t, aralık en fazla 33t, uçlardan pay. Sıra: önce uçlar, sonra ortalar, bağlantının iki tarafı dönüşümlü. Örnek: 8 mm plaka, 250 mm dikiş → 2 punta, 32 mm uzunluk. | dikişler | punta noktası, segment, sıra (mm) |
| **Erişilebilirlik** | Her punta için kalem eğimi × dönmesi kombinasyonlarını, hep aynı kol duruşunda deniyor. Kol, takım, parçalar ve masa çarpışmasız olmalı; en büyük boşluk seçiliyor. | puntalar, parçalar, takım modeli | punta başına kalem eğimi ve dönmesi |
| **Hareket ve işaret** | Puntalar arası geçiş: düz yol ya da RRT-Connect. Kalem ekseni boyunca iniş, kuvvet 1.5 N olunca duruyor. Nokta ya da çizgi, sonra geri çekiliyor. | kalem açıları, eklemler, kuvvet sensörü | temas noktası ve derinliği |
| **Doğrulama** | İşaretin kökten uzaklığını kalem dokunuşlarıyla, kameradan bağımsız ölçüyor. 2 Ekim: bütün puntalar ~2.5 mm içinde. | temaslar, kayıtlı yüzler | hata (mm) |

Ayrıntılar: algılama tarafı [perception_aciklama.md](perception_aciklama.md), robot tarafı
[admitans kontrol açıklama.md](<admitans kontrol açıklama.md>).
