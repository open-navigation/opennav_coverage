# F2C v2 — Bölüm B.6 + B.7 Uygulama Planı

> Bölüm A (zorunlu migration) tüm branch'lerde tamamlandı.
> Bu döküman yalnızca **B.6 (hız profili + yön)** ve **B.7 (path işleme)** özelliklerini kapsar.
> İkisi bağımsız; B.3→B.1→B.2 zincirine bağlı değil. Sırayla ya da paralel uygulanabilir.

---

## B.6 — Hız Profili (`velocity`) + Yön (`dir`) 🟢

### Ne Var, Ne Eksik?

`PathState` (F2C v2) artık iki yeni alan taşıyor:

```cpp
// fields2cover/types/PathState.h
struct PathState {
  // ... mevcut alanlar ...
  PathDirection dir {PathDirection::FORWARD};   // FORWARD=1, BACKWARD=-1
  double velocity {1.0};                        // m/s (PathPlanning tarafından doldurulur)
};
```

Mevcut `utils.hpp::toMsg(PathState)` ve `toNavPathMsg` bu iki alanı **tamamen yok sayıyor**.
`PathComponents.msg` da geri yönlü sürüş veya hız bilgisi taşımıyor.
Sonuç: geri viteste yapılması gereken dönüşler yanlış (ileri) yönde takip ediliyor;
hız profili kayıp.

### Değişecek Dosyalar

| Dosya | Değişiklik türü |
|---|---|
| `opennav_coverage_msgs/msg/PathComponents.msg` | Yeni alanlar ekleme |
| `opennav_coverage/include/opennav_coverage/utils.hpp` | `toNavPathMsg` + yeni helper |
| `opennav_coverage/src/coverage_server.cpp` | Yeni alanları doldurma |
| `opennav_row_coverage/src/row_coverage_server.cpp` | Yeni alanları doldurma |
| `opennav_coverage/test/test_utils.cpp` | Yeni test case'leri |

### Adım Adım

#### Adım 1 — `PathComponents.msg`'e iki alan ekle

```
# opennav_coverage_msgs/msg/PathComponents.msg
# (mevcut alanlardan sonra)

# Per-pose velocity (m/s) and direction flags, parallel to nav_path.poses.
# Populated only when generate_path=true. Empty when path is not generated.
float64[] velocities
bool[] is_backward
```

Bu iki dizi `nav_path.poses` ile paralel (aynı indeks = aynı nokta).

#### Adım 2 — `utils.hpp`'ye yardımcı fonksiyonlar ekle

`toNavPathMsg` dönüşü sırasında her `PathState`'ten velocity ve dir okunacak.
Mevcut `toNavPathMsg` imzası değişmeyecek — aşırı yük (overload) eklenecek:

```cpp
// Mevcut: nav_msgs::msg::Path döndürür
inline nav_msgs::msg::Path toNavPathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian,
  const float & pt_dist);

// YENİ: velocity + is_backward vektörlerini de doldurur
// out_velocities ve out_is_backward çıkış parametresi olarak alınır
inline nav_msgs::msg::Path toNavPathMsg(
  const Path & raw_path, const F2CField & field,
  const std_msgs::msg::Header & header, const bool is_cartesian,
  const float & pt_dist,
  std::vector<double> & out_velocities,
  std::vector<bool> & out_is_backward);
```

İç mantık: mevcut swath densify döngüsünde her pose push edilirken,
aynı `PathState`'in `velocity` ve `dir` değeri de paralel vektöre yazılır.
Turn pose'ları için de aynı işlem (turn'lerde velocity PathPlanning tarafından
genellikle cruise speed veya 0 olarak doldurulur).

```cpp
// Densify döngüsünde swath noktaları eklerken (mevcut while bloğunun içine):
out_velocities.push_back(path[i].velocity);
out_is_backward.push_back(path[i].dir == f2c::types::PathDirection::BACKWARD);

// Turn nokta eklerken:
out_velocities.push_back(path[i].velocity);
out_is_backward.push_back(path[i].dir == f2c::types::PathDirection::BACKWARD);
```

> **Dikkat:** Swath densify bloğu bir `while` döngüsüyle ara nokta ekliyor.
> Ara noktalar aynı `PathState`'ten türetildiği için aynı velocity/dir değerini alır —
> bu doğru davranış (bir swath boyunca velocity sabit).

#### Adım 3 — `coverage_server.cpp` + `row_coverage_server.cpp`'yi güncelle

Şu an (coverage_server.cpp:208-210):
```cpp
result->coverage_path =
  util::toCoveragePathMsg(path, master_field, header, cartesian_frame_);
result->nav_path = util::toNavPathMsg(
  path, master_field, header, cartesian_frame_, path_params_->pt_dist);
```

Sonra:
```cpp
result->coverage_path =
  util::toCoveragePathMsg(path, master_field, header, cartesian_frame_);
std::vector<double> velocities;
std::vector<bool> is_backward;
result->nav_path = util::toNavPathMsg(
  path, master_field, header, cartesian_frame_, path_params_->pt_dist,
  velocities, is_backward);
result->coverage_path.velocities =
  std::vector<float64_t>(velocities.begin(), velocities.end());
result->coverage_path.is_backward = is_backward;
```

> `row_coverage_server.cpp`'de aynı değişiklik (satır ~208).

### Test Stratejisi — B.6

**Mevcut testlere ek olarak** `test_utils.cpp`'e şu case'ler eklenir:

#### B.6-T1: Velocity ve is_backward doğrulama

```cpp
TEST(UtilsTests, TesttoNavPathMsgWithVelocity)
{
  // 2 SWATH + 1 TURN arası geçiş içeren path
  // SWATH velocity=2.0, TURN velocity=0.5, BACKWARD dir
  Path path;
  // ... state'leri elle kur ...

  std::vector<double> vels;
  std::vector<bool> dirs;
  auto nav_path = util::toNavPathMsg(path, F2CField(), header, true, 0.5, vels, dirs);

  EXPECT_EQ(nav_path.poses.size(), vels.size());
  EXPECT_EQ(nav_path.poses.size(), dirs.size());
  // Swath pose'larının velocity değeri beklenen değere eşit
  EXPECT_NEAR(vels[0], 2.0, 1e-6);
  // Backward flag'i kontrol
  EXPECT_TRUE(dirs[0]);
}
```

#### B.6-T2: Boş path → boş vektörler

```cpp
TEST(UtilsTests, TesttoNavPathMsgEmptyVelocity)
{
  Path empty_path;
  std::vector<double> vels;
  std::vector<bool> dirs;
  auto nav_path = util::toNavPathMsg(empty_path, F2CField(), header, true, 0.1, vels, dirs);
  EXPECT_TRUE(vels.empty());
  EXPECT_TRUE(dirs.empty());
}
```

#### B.6-T3: Forward-only path → is_backward tümü false

```cpp
TEST(UtilsTests, TesttoNavPathMsgForwardOnly)
{
  // Tüm state'ler FORWARD
  // dirs vektörünün tümü false olmalı
}
```

**Özel test dosyası gerekli mi?** Hayır. Mevcut `test_utils.cpp` yeterli;
yeni case'ler oraya eklenir.

---

## B.7 — Path İşleme (`discretize` / `populate` / `reduce` / `getTaskTime`) 🟢

### Ne Var, Ne Eksik?

F2C v2 `Path` yapısına 4 yeni metod eklendi:

```cpp
// fields2cover/types/Path.h
double getTaskTime() const;                // toplam süre: Σ(len/velocity) — velocity=0 → inf/NaN!
Path discretizeSwath(double step_size) const; // sadece SWATH state'leri böler, TURN'lere dokunmaz
Path& discretize(double step_size);        // populate(n) + reduce(step) → TÜM state'leri etkiler
Path& populate(int number_points = 100);   // sabit nokta sayısıyla spline doldur (TURN dahil)
Path& reduce(double min_dist_equal=0.1);   // çok yakın/paralel noktaları sil (in-place)
```

> ⚠️ **Graphify doğrulaması (Path.cpp:434):** `discretize()` aslında `populate()` + `reduce()`
> kombinasyonudur. `populate()` SWATH/TURN ayırt etmez — tüm path boyunca spline interpolasyon
> uygular. Bu yüzden `toNavPathMsg`'de **`discretize()` KULLANILMAYACAK**.
> Kullanılacak olan: `discretizeSwath(pt_dist)` — sadece SWATH segmentlerini böler,
> TURN state'lerini `else { addState(s); }` ile olduğu gibi geçirir. Mevcut elle yazılmış
> interpolasyonun birebir karşılığı budur.

**Mevcut sorun 1 (`toNavPathMsg`):** Swath densify mantığı utils.hpp'de elle yazılmış
(x0,y0→x1,y1 arası lineer interpolasyon, ~15 satır). `path.discretizeSwath(pt_dist)` bunu
F2C tarafında yapıyor — daha doğru, test edilmiş, TURN'lere dokunmuyor.

**Mevcut sorun 2:** Action result'ta `planning_time` (sunucu-tarafı hesap süresi) var,
ama **tahmin edilen yürütme süresi** (robotun sahadaki zamanı) yok.
`getTaskTime()` bunu Σ(len/velocity) formülüyle hesaplıyor.

**Mevcut sorun 3:** Turn'lerin dense F2C path'i zaten kalabalık olabilir;
`reduce(min_dist)` onu inceltebilir. Şu an bu adım hiç uygulanmıyor.

### Değişecek Dosyalar

| Dosya | Değişiklik türü |
|---|---|
| `opennav_coverage_msgs/action/ComputeCoveragePath.action` | Result'a `task_time` ekle |
| `opennav_coverage/include/opennav_coverage/utils.hpp` | `toNavPathMsg` yeniden yaz |
| `opennav_coverage/src/coverage_server.cpp` | `task_time` doldur |
| `opennav_coverage/src/path_generator.cpp` | (opsiyonel) `reduce` adımı |
| `opennav_coverage/test/test_utils.cpp` | Yeni test case'leri |

### Adım Adım

#### Adım 1 — Action result'a `task_time` ekle

```
# ComputeCoveragePath.action — result section
nav_msgs/Path nav_path
opennav_coverage_msgs/PathComponents coverage_path
builtin_interfaces/Duration planning_time
float64 task_time        # robot'un sahadaki tahmini yürütme süresi (saniye)
uint16 error_code
```

#### Adım 2 — `toNavPathMsg` içindeki el-yazımı interpolasyonu değiştir

**Mevcut (utils.hpp ~satır 229-260):**
```cpp
// Swaths come in pairs of start-end sequentially
if (i > 0 && path[i].type == PathSectionType::SWATH &&
  path[i - 1].type == PathSectionType::SWATH)
{
  const float & x0 = path[i - 1].point.getX();
  // ... lineer interpolasyon döngüsü (~12 satır) ...
} else {
  msg.poses.push_back(toMsg(path[i]));
}
```

**Sonra:**
```cpp
// path kopyasını discretizeSwath ile yoğunlaştır — sadece SWATH bölünür, TURN olduğu gibi kalır
Path path = raw_path;
if (!is_cartesian) {
  path = f2c::Transform::transformToPrevCRS(raw_path, field);
} else {
  path.moveTo(field.getRefPoint());
}
// ⚠️ discretize() DEĞİL — o populate()+reduce() ile TURN dahil her şeyi değiştirir
path = path.discretizeSwath(static_cast<double>(pt_dist));  // ← YENİ: F2C API, const metod

for (const auto & state : path) {
  msg.poses.push_back(toMsg(state));  // döngü koşulsuz, SWATH dense + TURN değişmedi
}
```

> **Neden `discretizeSwath`?** Graphify + Path.cpp kaynak doğrulamasıyla teyit edildi
> (L305-321): `discretizeSwath` yalnızca `type == SWATH` state'leri böler; diğerleri
> `else { addState(s); }` ile dokunulmadan geçer. `discretize()` (L434) ise `populate(n) +
> reduce(step)` — `populate` TURN dahil tüm state'lere spline uygular, davranış değişir.

> **B.6 ile entegrasyon:** `discretizeSwath` yeni bir `Path` döndürür (copy); her state'in
> `velocity` ve `dir` alanları kopyalanır (`state = s` ataması, Path.cpp L312). B.6 overload'u
> aynı düzeni sorunsuz kullanır.

#### ✅ UYGULANDI — Adım 3 — (Opsiyonel) `reduce` adımı ekle
> ## ✅ BU ADIM UYGULANDI (2026-07-10)
> `path_generator.cpp`'de `reduce_path_` + `reduce_min_dist_` parametreleri ve `path.reduce()` çağrısı eklendi;
> `coverage_server.cpp` dinamik parametre callback'i güncellendi. Default `false`.

#### (Opsiyonel) `reduce` adımı ekle

`path_generator.cpp`'de path oluşturulduktan sonra:
```cpp
// Opsiyonel: turn'lerde çok yakın noktaları sil
// (varsayılan 0.1m eşiği genellikle güvenli)
if (reduce_path_) {   // yeni parametre: reduce_path (bool, default false)
  path.reduce(reduce_min_dist_);  // reduce_min_dist (double, default 0.1)
}
```

Bu adım **opsiyonel** — parametre olmadan şimdilik atlanabilir; B.7 için zorunlu değil.

#### Adım 4 — `coverage_server.cpp`'de `task_time` doldur

> ⚠️ **Graphify doğrulaması (Path.cpp:54-56):** `getTaskTime()` = `Σ(s.len / s.velocity)`.
> `velocity` default 1.0 ama F2C bunu garantilemez — caller'da guard şart.

```cpp
// path generate edildiğinde (generate_path == true dalında)
const double task_time = path.getTaskTime();
result->task_time = std::isfinite(task_time) ? task_time : 0.0;
```

`std::isfinite` guard'ı: velocity=0 olan herhangi bir state `inf` üretir; bunu 0.0'a dönüştürüyoruz
(hız bilinmiyor → süre bilinmiyor anlamında). `<cmath>` zaten utils.hpp/coverage_server'da mevcut.

`row_coverage_server.cpp`'de aynı guard ile aynı ekleme yapılacak (satır ~222).

### Test Stratejisi — B.7

#### B.7-T1: `discretizeSwath` → nokta sayısı deterministik

```cpp
TEST(UtilsTests, TesttoNavPathMsgDiscreteSwathPointCount)
{
  // discretizeSwath deterministik: n_steps = round(fabs(1.0/0.1)) = 10 (Path.cpp:309)
  Path path;
  PathState s;
  s.type = PathSectionType::SWATH;
  s.point = Point(0.0, 0.0);
  s.len = 1.0;
  s.angle = 0.0;
  path.addState(s);

  auto msg = util::toNavPathMsg(path, F2CField(), header, true, 0.1f);
  EXPECT_EQ(msg.poses.size(), 10u);  // round(1.0/0.1)=10, deterministik — aralık değil
}
```

#### B.7-T2b: TURN state'leri `discretizeSwath` sonrası değişmemeli

```cpp
TEST(UtilsTests, TesttoNavPathMsgTurnUnchanged)
{
  // 10 TURN state → toNavPathMsg'de discretizeSwath bunlara dokunmamalı
  // sonuç poses.size() == 10 olmalı (mevcut TesttoNavPathMsg testindeki gibi)
  Path path;
  path.getStates().resize(10);
  for (auto & state : path.getStates()) {
    state.type = f2c::types::PathSectionType::TURN;
  }
  auto msg = util::toNavPathMsg(path, F2CField(), header, true, 0.1f);
  EXPECT_EQ(msg.poses.size(), 10u);  // TURN'ler olduğu gibi, ek nokta yok
}
```

Bu test mevcut `TesttoNavPathMsg` ile zaten örtüşüyor — yeni implementasyonun
TURN davranışını bozmadığını teyit eden **regresyon testi** olarak eklenecek.

#### B.7-T2: `getTaskTime` — normal durum

```cpp
TEST(UtilsTests, TestGetTaskTime)
{
  Path path;
  PathState s;
  s.len = 5.0;
  s.velocity = 2.0;   // beklenen katkı: 5/2 = 2.5 sn
  path.addState(s);

  EXPECT_NEAR(path.getTaskTime(), 2.5, 0.01);
}
```

#### B.7-T2c: velocity=0 guard — coverage_server çıktısı finite olmalı

```cpp
TEST(UtilsTests, TestGetTaskTimeZeroVelocity)
{
  // velocity=0 → getTaskTime() = inf; coverage_server guard bunu 0.0'a çevirmeli
  Path path;
  PathState s;
  s.len = 1.0;
  s.velocity = 0.0;   // geçersiz — Path.cpp:56 s.len/s.velocity = inf üretir
  path.addState(s);

  // F2C API doğrudan inf üretir (guard yok)
  EXPECT_FALSE(std::isfinite(path.getTaskTime()));
  // coverage_server'daki guard: std::isfinite(task_time) ? task_time : 0.0
  const double guarded = std::isfinite(path.getTaskTime()) ? path.getTaskTime() : 0.0;
  EXPECT_NEAR(guarded, 0.0, 1e-9);
}
```

#### B.7-T3: Boş path → `getTaskTime` == 0

```cpp
TEST(UtilsTests, TestGetTaskTimeEmptyPath)
{
  Path empty;
  EXPECT_NEAR(empty.getTaskTime(), 0.0, 1e-9);
}
```

#### B.7+B.6 Entegrasyon Testi: densify sonrası poses / vels / dirs boyut uyumu

B.6 ve B.7 aynı `toNavPathMsg` overload'unda buluşuyor:
`discretizeSwath(pt_dist)` → yeni Path (ara nokta sayısı artık farklı) → velocity döngüsü
aynı state'leri okuyor. Boyut uyuşmazlığı sessiz UB veya index-out-of-range üretebilir.

```cpp
TEST(UtilsTests, TesttoNavPathMsgDensifyVelocitySizeMatch)
{
  // 1m SWATH (pt_dist=0.1 → 10 nokta) + 2 TURN state
  Path path;
  PathState swath;
  swath.type = PathSectionType::SWATH;
  swath.len = 1.0;
  swath.velocity = 1.5;
  swath.dir = PathDirection::FORWARD;
  path.addState(swath);

  PathState turn;
  turn.type = PathSectionType::TURN;
  turn.velocity = 0.5;
  turn.dir = PathDirection::BACKWARD;
  path.addState(turn);
  path.addState(turn);

  std::vector<double> vels;
  std::vector<bool> dirs;
  auto nav_path = util::toNavPathMsg(path, F2CField(), header, true, 0.1f, vels, dirs);

  // Boyut uyumu — densify sonrası kaç pose varsa o kadar vel/dir olmalı
  EXPECT_EQ(nav_path.poses.size(), vels.size());
  EXPECT_EQ(nav_path.poses.size(), dirs.size());
  // SWATH noktaları forward, TURN noktaları backward
  EXPECT_FALSE(dirs[0]);   // SWATH → FORWARD
  EXPECT_TRUE(dirs.back()); // TURN → BACKWARD
}
```

**Özel test dosyası gerekli mi?** Hayır. Tüm case'ler `test_utils.cpp`'e eklenir.

---

## row_coverage_server Entegrasyon Testleri

Plan row_coverage_server.cpp'yi değiştiriyor (B.6 Adım 3: velocities/is_backward,
B.7 Adım 4: task_time). `opennav_row_coverage/test/` altında B.6/B.7'ye ait hiçbir
doğrulama yok — server doğru doldursa bile test yoksa fark edilmez.

Eklenecek yer: `opennav_row_coverage/test/test_server.cpp` (mevcut dosya).

```cpp
// row_coverage_server: velocities + task_time dolduruldu mu?
TEST_F(RowCoverageServerTest, TestRowServerVelocityAndTaskTime)
{
  // Mevcut test fixture'ını kullanarak coverage path action gönder
  // (deterministik, küçük poligon — test_server.cpp'deki mevcut örnekle aynı pattern)

  // result->coverage_path.velocities boş olmamalı (generate_path=true ise)
  EXPECT_FALSE(result->coverage_path.velocities.empty());
  // Her velocity değeri >= 0
  for (const auto & v : result->coverage_path.velocities) {
    EXPECT_GE(v, 0.0);
  }
  // task_time finite ve > 0 olmalı (path var, velocity > 0)
  EXPECT_TRUE(std::isfinite(result->task_time));
  EXPECT_GT(result->task_time, 0.0);
  // Boyut uyumu
  EXPECT_EQ(result->nav_path.poses.size(), result->coverage_path.velocities.size());
  EXPECT_EQ(result->nav_path.poses.size(), result->coverage_path.is_backward.size());
}
```

> Fixture ve action gönderi mekanizması için mevcut `test_server.cpp`'deki
> `TEST_F(RowCoverageServerTest, ...)` örneklerine bak — pattern aynı.

---

## Uygulama Sırası

```
B.7 önce → B.6 sonra
```

**Neden B.7 önce?**
- `toNavPathMsg`'yi B.7'de yeniden yazıyoruz (discretize entegrasyonu).
- B.6'nın overload'u aynı yeniden yazılmış gövde üzerinde çalışır.
- İkisini ters sırayla yaparsak B.6 overload'unu iki kez dokunmak gerekir.

### Sıra

1. **B.7 Adım 1** — action'a `task_time` ekle (mesaj değişikliği; build kırar, önce tamamla)
2. **B.7 Adım 2** — `toNavPathMsg` içini discretize ile değiştir + test B.7-T1
3. **B.7 Adım 3** — (opsiyonel) `reduce` parametresi
4. **B.7 Adım 4** — coverage_server + row_coverage_server `task_time` doldur + test B.7-T2/T3
5. **B.6 Adım 1** — `PathComponents.msg`'e velocity/is_backward ekle
6. **B.6 Adım 2** — `toNavPathMsg` overload'u ekle + test B.6-T1/T2/T3
7. **B.6 Adım 3** — coverage_server + row_coverage_server yeni overload'u kullan
8. Build: `colcon build --packages-select opennav_coverage_msgs opennav_coverage opennav_row_coverage`
9. Test: `colcon test --packages-select opennav_coverage opennav_row_coverage`

---

## Genel Test Akışı

```
colcon test → tüm mevcut 207 test geçiyor mu? (regresyon kontrolü)
         ↓ geçti
yeni test case'leri geçiyor mu?
         ↓ geçti
RViz davranışsal kontrol — gerekmez (B.6+B.7 utils katmanında, plan geometrisi değişmiyor)
         ↓
PR (main branch)
```

> **RViz neden gerekmiyor?** B.6 ve B.7, plan geometrisini değiştirmiyor —
> swath/turn konumları aynı. `velocity`/`dir`/`task_time` yeni alanlar;
> mevcut path görüntüsünü bozmaz. Sayısal doğrulama unit test'lerle yeterli.

---

## Riskler / Dikkat Edilecekler

> Aşağıdaki tablo graphify + Path.cpp kaynak doğrulamasıyla güncellenmiştir.

| Risk | Durum | Karar |
|---|---|---|
| `discretize()` TURN state'leri de değiştirir | ✅ **DOĞRULANDI** (Path.cpp:434: `populate()+reduce()`, TURN dahil) | `discretizeSwath()` kullanılacak — TURN'lere dokunmaz (L305-321) |
| `getTaskTime()` velocity=0 → div-by-zero | ✅ **DOĞRULANDI** (Path.cpp:56: `s.len/s.velocity`, guard yok) | Caller'da `std::isfinite()` guard zorunlu (Adım 4); test B.7-T2c eklendi |
| `PathComponents.msg` yeni alan → client kırar | ✅ **GÜVENLİ** (graphify: BT node struct geçiyor, alan tek tek okumaz) | ROS 2 yeni alan eklemeye izin veriyor; boş liste backwards-compatible |
| `row_coverage_server` değişiklik eksik kalır | ✅ **SİMETRİK** (graphify: her iki server aynı util çağrılarını kullanıyor) | Adım 4 ve B.6 Adım 3'ü her zaman çift olarak yap |
