# Fields2Cover v2 — Güncellenecek Kısımlar & Yeni Özellik Eklenecek Kısımlar

> **Amaç:** v2'nin getirdiği ve mevcut kodda henüz kullanılmayan yeni yetenekler ve bunların kod tabanına nereden bağlanacağı.
>
> **Not:** Zorunlu migration (Bölüm A — A.1–A.13) tüm branch'lerde tamamlandı ve upstream'e merge edildi (humble-v2 #105, jazzy #109, main #110). Bu döküman yalnızca **Bölüm B — Yeni Özellikler**'i içerir.
>
> ⚠️ Satır referansları `main` branch kaynağına göredir.

---

# YENİ ÖZELLİKLER — v2'ye Geçiş Sonrası Geliştirmeler

v2'nin getirdiği ve mevcut kodda **henüz kullanılmayan** yetenekler. Migration tamamlandıktan sonra değer katacak geliştirmeler.

## B.0 Özet — Yeni Özellik Fırsatları

| # | Yeni özellik | Değer | Bağlanacağı yer | Efor |
|---|--------------|-------|-----------------|:--:|
| B.1 | Çok-hücreli TSP rota (`RoutePlannerBase`) | Headland üzerinden geçen gerçek rota; çoklu poligon | route_generator | 🔴 Büyük |
| B.2 | `F2CRoute` + bağlantılar (connections) | Şeritler arası gerçek geçiş yolları | route + path + utils | 🟠 Orta |
| B.3 | Decomposition (non-convex tarla) | L-şekilli tarlada şerit sayısını ~%70 azaltma | yeni modül + server | 🟠 Orta |
| B.4 | `NSwathModified` objektifi | Hızlı şerit sayısı objektifi | swath_generator + enum | 🟢 Küçük |
| B.5 | `HL_SWATH` tipi + headland şeritleri | Headland'i de sürülebilir yol olarak işle | utils PathState makinesi | 🟢 Küçük — B.3 sonrası |
| B.6 | Hız profili (`velocity`) + yön (`dir`) | ROS çıktısına hız/geri-sürüş bilgisi | utils → nav_msgs | 🟢 Küçük |
| B.7 | Path işleme (`discretize`/`populate`/`reduce`/`getTaskTime`) | Daha temiz/yoğun yol + süre tahmini | utils | 🟢 Küçük |
| B.8 | Facade (`planCovPath`/`Options`) | Hızlı "tek çağrı" alternatif hat | yeni opsiyonel mod | 🟠 Orta |
| B.9 | Başlangıç/bitiş noktası sabitleme | Robotun mevcut konumundan başlatma | route_generator | 🟢 Küçük |

### Özellikler Arası Bağımlılık

```
B.3  ✅ tamamlandı
B.4  ✅ tamamlandı
B.5  ✅ tamamlandı
B.6  ✅ tamamlandı
B.7  ✅ tamamlandı

B.3 Decomposition
    ├─► B.5 HL_SWATH desteği   (B.3 üretir, B.5 tüketir — aynı PR'da)
    └─► B.1 TSP Rota            (çok-hücreli çıktıyı rotalandırmak için TSP şart)
            └─► B.2 Bağlantılar (F2CRoute olmadan TSP çıktısı path'e dönüşemez)
                    └─► B.9 Start/End  (setStartAndEndPoint TSP planlayıcısının metodu)

B.8 (bağımsız ama B.3/B.1 varsa anlam kazanır — sonraya bırakılabilir)
```

**Bağımsız (hemen yapılabilir):** B.4, B.8  
**B.3'e bağımlı:** B.5 (B.3 ile birlikte)  
**Zincirine bağımlı:** B.1 → B.2 → B.9

> ⚠️ B.1+B.2 birlikte başlanmalı; biri olmadan diğeri yarım kalır. B.5 B.3 olmadan
> ölü kod olur — end-to-end test edilemez. B.8 (facade) B.3/B.1 varsa yetersiz
> kalır, granüler hat şart.

---

## B.1 Çok-Hücreli TSP Rota Planlama — `RoutePlannerBase` 🔴

**Şu an:** [route_generator.cpp](opennav_coverage/src/route_generator.cpp) yalnızca **tek-hücre** desen sıralayıcıları kullanıyor (`BoustrophedonOrder`/`SnakeOrder`/`SpiralOrder`/`CustomOrder` → `genSortedSwaths`). Bu, şeritleri sadece bir sıraya dizer; hücreler arası veya headland üzerinden gerçek navigasyonu hesaba katmaz.

**v2 yeniliği:** `f2c::rp::RoutePlannerBase::genRoute(cells, swaths_by_cells, ...)` — OR-Tools TSP tabanlı, **headland'ler üzerinden geçen** rota planlar; çoklu poligon/hücre destekler.

```cpp
f2c::rp::RoutePlannerBase rp;
F2CRoute route = rp.genRoute(mainland_cells, swaths_by_cells);
// opsiyonel: rp.setStartAndEndPoint(robot_current_pos);  // bkz. B.9
```

**Bağlanacağı yer:**
- Yeni bir `RouteType::TSP` (veya `SHORTEST_ROUTE`) enum değeri → [types.hpp:81](opennav_coverage/include/opennav_coverage/types.hpp#L81)
- `RouteGenerator`'a `RoutePlannerBase` dalı ekle; mevcut desen sıralayıcılar korunabilir (geriye uyumluluk).
- Çıktı `F2CSwaths` değil **`F2CRoute`** olur → utils dönüşüm katmanı `F2CRoute` overload'u ister (Bölüm B.2).

> 📌 Bu özellik, opennav_coverage'ı tek-poligon kapsamadan **çok-hücreli/headland-bağlantılı** planlamaya taşır. En büyük mimari kazanım, en yüksek efor.

---

## B.2 `F2CRoute` ve Şerit-Arası Bağlantılar 🟠

**Şu an:** Pipeline `Swaths → Path` (dönüşler `PathPlanning` tarafından naif üretilir). `F2CRoute`'un tuttuğu **connections** (şeritler arası geçiş yolları) kullanılmıyor.

**v2 yeniliği:** `F2CRoute` hem şeritleri hem aralarındaki bağlantı yollarını (`getConnections()` → `MultiPoint`) tutar; `asLineString()`, `length()`, `startPoint()/endPoint()` sağlar. `PathPlanning::planPath(robot, F2CRoute, TurningBase&)` overload'u rotayı akıcı sürülebilir yola çevirir.

**Bağlanacağı yer:**
- [utils.hpp](opennav_coverage/include/opennav_coverage/utils.hpp) — `toCoveragePathMsg`/`toNavPathMsg` için `F2CRoute` overload'ları ekle.
- [path_generator.cpp](opennav_coverage/src/path_generator.cpp) — `planPath(robot, route, curve)` overload'unu kullan.
- Headland halkaları üzerinden geçen bağlantılar, daha gerçekçi (engele/sınıra saygılı) geçişler verir.

---

## B.3 Decomposition — Non-Convex Tarla Ayrıştırma 🟠

**Şu an:** Hiç yok. Mevcut hat tarlayı tek convex bölge gibi işliyor; L/U-şekilli tarlalarda swath sayısı patlıyor.

**v2 yeniliği:** `f2c::decomp::TrapezoidalDecomp` / `BoustrophedonDecomp` — non-convex tarlayı convex alt hücrelere böler (tutorial örneğinde şerit sayısı 84→26).

```cpp
f2c::decomp::TrapezoidalDecomp decomp;
decomp.setSplitAngle(0.5 * M_PI);
F2CCells decomposed = decomp.decompose(cells);   // headland'den ÖNCE
```

**Bağlanacağı yer:**
- Yeni opsiyonel aşama → [coverage_server.cpp:188](opennav_coverage/src/coverage_server.cpp#L188) civarı (headland'den önce).
- Yeni `DecompType` enum + parametre/action alanı.
- ⚠️ Facade'de decomposition **devre dışı**; granüler API'yi (`f2c::decomp::*`) doğrudan çağırmak gerekir.
- Çıktı çok-hücre olacağı için B.1 (çok-hücreli rota) ile birlikte en verimli.

### B.3.viz — Decomposed hücrelerin görselleştirmesi (ertelendi)

> **Kaynak:** B.3+B.5 uygulama planından (`F2C_V2_B3_B5_UYGULAMA_PLANI.md`) çıkarılıp buraya taşındı.
> B.3'ün çekirdeği (decompose + swath) buna bağımlı değil; ayrı bir görselleştirme geliştirmesidir.

**Şu an:** Mevcut `Visualizer::visualize(Field, Field, ...)` tek hücrelik `Field` alır;
`planning_field_pub_` decomposed çok-hücreyi gösteremez. B.3 çekirdeğinde decomp yolunda
`field_no_headland == field` verilerek toplam sınır + swath'lar zaten gösteriliyor — ama **her
sub-cell ayrı** gösterilmiyor.

**Geliştirme:** Yeni bir `MarkerArray` publisher (`coverage_server/decomposed_fields`) + her
sub-cell'i ayrı `LINE_STRIP` marker olarak yayınlayan bir overload.

**Bağlanacağı yer:** [visualizer.hpp](opennav_coverage/include/opennav_coverage/visualizer.hpp) /
[visualizer.cpp](opennav_coverage/src/visualizer.cpp) + [coverage_server.cpp:234](opennav_coverage/src/coverage_server.cpp#L234) visualize çağrısı.

```cpp
// visualizer.cpp — yeni overload (özet). ⚠️ frame_id HARDCODED "map" DEĞİL;
// server'ın CRS/goal'dan gelen header.frame_id'sini kullan (mevcut kodla tutarlı).
void Visualizer::visualize(
  const Field & total_field, const F2CCells & decomposed_cells,
  const Point & ref_pt, const nav_msgs::msg::Path & nav_path,
  const Swaths swaths, const std_msgs::msg::Header & header)
{
  // Ortak davranışı tekrar yazma: ilk hücreyle mevcut tek-hücre visualize()'ı çağır
  if (decomposed_cells.size() > 0) {
    visualize(total_field, decomposed_cells.getGeometry(0), ref_pt, nav_path, swaths, header);
  }

  if (decomposed_cells_pub_->get_subscription_count() > 0) {
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    for (size_t i = 0; i < decomposed_cells.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;                 // ← frame_id server'dan gelir, "map" hardcode YOK
      marker.ns = "decomposed_cells";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.15;
      marker.color.g = 1.0;  // yeşil — sınır(beyaz)/swath(mavi)'dan ayırt edici
      marker.color.a = 1.0;
      Polygon boundary = decomposed_cells.getGeometry(i).getGeometry(0);
      for (size_t j = 0; j < boundary.size(); ++j) {
        marker.points.push_back(
          util::pointToPoint32(util::toMsg(boundary.getGeometry(j) + ref_pt)));
      }
      marker_array->markers.push_back(marker);
    }
    decomposed_cells_pub_->publish(std::move(marker_array));
  }
}
```

- `activate()`/`deactivate()`'e `decomposed_cells_pub_` create/reset ekle; header'a
  `visualization_msgs/msg/marker_array.hpp` include'u.
- Server tarafında decomp yolunda bu overload çağrılır; non-decomp yolu **değişmez**.
- ⚠️ **Kritik düzeltme:** İlk taslakta `marker.header.frame_id = "map"` hardcoded idi; server'ın
  geri kalanı dinamik `frame_id` (CRS/goal) kullandığından bu bir hataydı — yukarıda `marker.header = header`
  ile giderildi.

---

## B.4 `NSwathModified` Objektifi 🟢

**Şu an:** [swath_generator.cpp:78-82](opennav_coverage/src/swath_generator.cpp#L78) yalnızca `SwathLength`/`NSwath`/`FieldCoverage` sunuyor.

**v2 yeniliği:** `f2c::obj::NSwathModified` — `NSwath`'a hızlı yaklaşım (`isFastCompAvailable=true`), şerit sayısını geometriden hızlı hesaplar. Facade'de **varsayılan** (`N_SWATH_MOD`).

**Bağlanacağı yer:**
- [types.hpp:60](opennav_coverage/include/opennav_coverage/types.hpp#L60) `SwathType`'a `NUMBER_FAST` ekle.
- [swath_generator.cpp:74](opennav_coverage/src/swath_generator.cpp#L74) `createObjective`'e `case` ekle.
- Brute-force açı taramasını belirgin hızlandırır.

---

## B.5 `HL_SWATH` Tipi + Headland Şeritleri 🟢

**Şu an:** [utils.hpp:145-196](opennav_coverage/include/opennav_coverage/utils.hpp#L145) yol durum makinesi yalnızca `PathSectionType::SWATH` ve `TURN` ayırıyor.

**v2 yeniliği:** `PathSectionType::HL_SWATH=3` — headland içindeki şeritleri ayrı işaretler. Ayrıca `ConstHL::generateHeadlandSwaths(...)` headland halkalarını sürülebilir şerit olarak üretir.

**Bağlanacağı yer:**
- utils yol makinesine `HL_SWATH` dalı ekle (örn. headland şeritlerinde alet davranışı farklı).
- `Swath::getType()` (MAINLAND/HEADLAND) ile mainland/headland ayrımı yapılabilir.

---

## B.6 Hız Profili ve Sürüş Yönü 🟢

**Şu an:** [utils.hpp:70-78](opennav_coverage/include/opennav_coverage/utils.hpp#L70) sadece `point` + `angle` → `PoseStamped` üretiyor; hız ve yön bilgisi atılıyor.

**v2 yeniliği:** `PathState` artık `velocity` (segment hızı) ve `dir` (`FORWARD=1`/`BACKWARD=-1`) tutuyor; `Robot::setCruiseVel/setTurnVel` ile beslenir.

**Bağlanacağı yer:**
- ROS çıktısına hız profili (örn. ayrı bir hız dizisi veya `nav2_msgs` uzantısı) ekle.
- `dir == BACKWARD` segmentlerini işaretle → kontrolcü geri sürüşü desteklemiyorsa uyar/`DUBINS` zorla.
- `Robot` parametrelerine `cruise_speed`/`turn_speed` ROS paramları ekle (A.2 ile birlikte).

---

## B.7 Path Son-İşleme: `discretize` / `populate` / `reduce` / `getTaskTime` 🟢

**Şu an:** [utils.hpp:209-263](opennav_coverage/include/opennav_coverage/utils.hpp#L209) `toNavPathMsg` yoğunlaştırmayı **elle** yapıyor.

**v2 yeniliği (zaten v1'de de vardı, v2'de stabil):**
- `path.discretize(step)` / `discretizeSwath(step)` — sabit adımla örnekleme
- `path.populate(n)` — spline ile n noktaya interpolasyon
- `path.reduce(min_dist)` — yakın/yinelenen noktaları siler
- `path.getTaskTime()` — Σ len/velocity, **tahmini görev süresi**

**Bağlanacağı yer:**
- `toNavPathMsg`'deki elle yoğunlaştırmayı `discretize`/`populate` ile sadeleştir.
- `getTaskTime()` → action feedback/result'a görev süresi tahmini olarak eklenebilir.

---

## B.8 Facade (`planCovPath` / `Options`) — Hızlı Alternatif Hat 🟠

**Şu an:** Granüler hat (headland→swath→route→path ayrı sınıflar). Tam kontrol verir ama kod uzun.

**v2 yeniliği:** `f2c::planCovPath(robot, field, opt, local_crs)` — tüm hattı tek çağrıda; `f2c::Options` ile aşama seçimi. `local_crs` bayrağı opennav'ın kartezyen/GPS modlarına birebir karşılık gelir.

**Bağlanacağı yer:**
- "Basit mod" olarak opsiyonel bir hızlı yol (örn. parametre ile granüler vs. facade seçimi).
- ⚠️ Decomposition ve özel sıralama facade'de kısıtlı → gelişmiş kullanım için granüler hat kalmalı. Bu yüzden facade'i **tam değişim değil, ek seçenek** olarak düşün.

---

## B.9 Başlangıç/Bitiş Noktası Sabitleme 🟢

**v2 yeniliği:** `RoutePlannerBase::setStartAndEndPoint(F2CPoint)` — rotayı robotun **mevcut konumundan** başlatır/bitirir.

**Bağlanacağı yer:**
- Action goal'a "robot mevcut pozu"nu (TF'den) geçir → daha doğal planlar.
- B.1 ile birlikte gelir (TSP rota planlayıcının metodu).

---

## Sıralama Önerisi (Bölüm B)

1. **Önce migration (Bölüm A) tamamlanmalı** — yeni özellikler v2 API'si üzerine kurulur.
2. Düşük eforlu hızlı kazanımlar: **B.4** (NSwathModified), **B.6** (hız/yön), **B.7** (path işleme), **B.5** (HL_SWATH).
3. Orta: **B.2** (Route+connections), **B.8** (facade), **B.3** (decomposition).
4. Büyük mimari: **B.1** (çok-hücreli TSP) + **B.9** (start/end) — en yüksek değer, en yüksek efor; B.2/B.3 ile birlikte planla.

---

*Bu döküman audit + iki F2C v2 referans belgesinin sentezidir. `🔶 doğrula` işaretli satırlar kurulu v2 başlıklarına karşı son kez teyit edilmelidir. Mevcut-kod satır referansları `humble` branch kaynağından doğrulanmıştır.*
