

### Kartezyen Admitansın Matematiği

Admitans kontrolü, robotun ucunu kütlesi ($M$), sönümlemesi ($D$) ve yay katsayısı ($K$) olan sanal bir mekanik sistem gibi modellendirir. Kartezyen uzayın 6 serbestlik derecesinin (X, Y, Z, Rx, Ry, Rz) her birine farklı $M, D, K$ değerleri atanabilir.

Kum çizimi senaryosunda:

* **X ve Y Eksenleri:** Çizim yörüngesinden sapmaması için çok yüksek yay katsayısına (Stiffness - $K$) sahip olmalıdır. Robot bu eksenlerde bir duvar kadar rijit davranır.
* **Z Ekseni (Yüzeye Dik Eksen):** Kumun veya metalin eğimini tolere edebilmesi için sıfır veya çok düşük bir yay katsayısına sahip olmalı, ancak belirli bir sönümleme ($D$) ile hedeflenen kuvveti ($F_{hedef}$) takip etmelidir.

Denklem şu şekildedir:


$$F_{okunan} - F_{hedef} = M \Delta \ddot{X}_z + D \Delta \dot{X}_z + K \Delta X_z$$

Sensörden okunan kuvvet, hedeflediğiniz kuvvetten farklıysa (örneğin robot kuma fazla dalmışsa), bu denklem Z ekseninde yukarı doğru bir $\Delta \dot{X}_z$ (hız) veya $\Delta X_z$ (pozisyon düzeltmesi) üretir. Bu düzeltme, X ve Y'deki çizim yörüngesinin üzerine eklenerek robota gönderilir.

Aşağıdaki simülasyon, bu sanal yayın ve sönümleyicinin engebeli bir yüzeyde (kum/metal) nasıl davrandığını ve parametrelerin kuvvet takibini nasıl etkilediğini test etmeniz için hazırlanmıştır.

### ROS 2'de Uygulama Mimarisi Seçenekleri

Bu sistemi ROS 2'ye entegre ederken önünüzde iki ana yol bulunmaktadır:


**Yol 2: MoveIt 2 Servo ile Özel (Custom) Node Yazmak**

* **Nasıl Çalışır:** Python veya C++ ile kendi ROS 2 düğmenizi (Node) yazarsınız. Bu düğme UR5e'nin `/force_torque_sensor_broadcaster/wrench` topic'ine abone olur. Ayrıca CAM'den gelen X-Y çizim yörüngesini okur. Sizin kodunuz, admitans formülünü (yukarıdaki denklem) 100Hz veya 500Hz'lik bir zamanlayıcı (timer callback) içinde hesaplar ve Z ekseni hızını bulur. Son olarak, elde edilen X, Y ve Z hızlarını bir `TwistStamped` mesajı olarak birleştirip **MoveIt Servo**'nun (eski adıyla jog_arm) komut topic'ine basar.
* **Avantajı:** Tüm kontrol matematiği sizin elinizdedir. İstediğiniz an yapay zeka (SAM3) entegrasyonu yapabilir, sınır koşullarına göre K ve D parametrelerini milisaniyeler içinde kod içinden değiştirebilirsiniz. Singularity (tekillik) ve çarpışma önleme işlemlerini MoveIt Servo arka planda halleder.

Kum üzerine çizim yapan sisteminizi metale taşıdığınızda, tek değişecek olan şey Admitans parametreleriniz (kum yumuşak olduğu için farklı, metal sert olduğu için farklı sönümleme gerektirir) ve CAM yörüngeniz olacaktır. ROS 2 mimarinizi bu modülerlikte kurarsanız, tahribatsız test aşamasından lazer kaynağa geçişiniz sadece kodda parametre değiştirmek kadar kolay olacaktır.