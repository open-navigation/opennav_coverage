# F2C v2 — Bölüm B.9 Uygulama Planı (Başlangıç/Bitiş Noktası Sabitleme)

> Bölüm A (zorunlu migration) tüm branch'lerde tamamlandı ve upstream'e merge edildi.
> **Tamamlanan Bölüm B maddeleri:** B.1 (TSP rota), B.2 (F2CRoute + connections — refactor'la
> bitti, aşağıda §0.1/§0.2), B.3 (decomposition), B.4 (NSwathModified), B.5 (HL_SWATH),
> B.6 (hız/yön), B.7 (path işleme).
> **Bu döküman yalnızca B.9'u kapsar.** B.8 (facade) ayrı dosyada: `F2C_V2_B8_UYGULAMA_PLANI.md`.
>
> Kod satır/imza referansları mevcut `Feature/umutc/f2c-enhancements-main` HEAD'ine göredir.

---

## 0.1 KÖKLÜ REFACTOR — B.9'u Etkileyen Mimari (git geçmişinden)

B.1/B.3 ilk atomik commit'lerinden sonra upstream'e giden hâlde yeniden yazıldı. B.9'u
doğrudan etkileyen commit'ler:

- `ceaa928 Add RouteMethod abstraction` + `50d26e9 Route all modes through RouteMethod, dropping is_tsp branching`
- `390a051 Remove dead overloads left behind by the F2CRoute unification`
- `5a31340 B.1: fix multi-cell TSP OOM via per-cell genRoute + route stitching`
- `67d3ebd Make the global-route swath cap a parameter`

Etkileri:

1. **`is_tsp` dallanması SİLİNDİ.** `coverage_server.cpp`'de artık tek yol var:
   `generateRoute(route_cells, swaths_by_cells, goal->route_mode) → F2CRoute` (`coverage_server.cpp:237`).
2. **`RouteMethod` soyutlaması** (`route_method.{hpp,cpp}`): `RouteMethod::plan(cells, swaths_by_cells,
   settings) → F2CRoute` saf sanal; `SwathOrderMethod` (desen sıralayıcılar) ve `TspRouteMethod`
   (OR-Tools `RoutePlannerBase`) override eder. `generateRouteTSP(...)` **kaldırıldı**.
3. **`TspRouteMethod::plan`** (`route_method.cpp:51`) iki yol izler:
   - `total_swaths <= max_swaths_for_global_route_` → **tek global** `rp.genRoute(cells, sbc, ...)`.
   - aksi halde **her hücre ayrı** `rp.genRoute` + rotalar düz-çizgi köprüyle dikilir. Eşik
     `max_swaths_for_global_route` **sunucu parametresi**.

> **B.9 için sonuç:** `setStartAndEndPoint`, artık var olmayan `generateRouteTSP`'a değil,
> `TspRouteMethod::plan`'a girmeli. **Per-cell stitch yolunda birden fazla `rp` var** →
> start/end'in nasıl uygulanacağı bir tasarım kararı (Adım 3).

## 0.2 Neden yalnızca B.9 kaldı?

- **B.2 kodu refactor ile bitti:** tüm modlar `F2CRoute` döndürüyor,
  `toCoveragePathMsg(const F2CRoute&, ...)` overload'u var (`utils.hpp:121`),
  `generatePath(const F2CRoute&, PathMode)` connection'ları path'e katıyor. Mevcut `test_route.cpp`
  bu yolu kapsıyor. Ayrı bir B.2 iş kalemi **yok**.
- **B.8 (facade)** ayrıldı — `F2C_V2_B8_UYGULAMA_PLANI.md`. Opsiyonel; decomposition'ı desteklemez.

**F2C v2 API teyidi (repodaki `Fields2Cover/` kaynağından):**
`void f2c::rp::RoutePlannerBase::setStartAndEndPoint(const F2CPoint& p);` → **tek nokta**
(start == end; rota bu noktadan başlar ve döner); dahili `std::optional<F2CPoint> r_start_end;`;
`genRoute(...)`'tan **önce** çağrılmalı.

---

## B.9 — Başlangıç/Bitiş Noktası Sabitleme 🟢

### Ne Var, Ne Eksik?
`TspRouteMethod::plan` her `RoutePlannerBase rp;`'yi doğrudan `rp.genRoute(...)`'a veriyor; robot
konumu hesaba katılmıyor. Robotun **mevcut konumundan** başlayıp oraya dönen rota için
`rp.setStartAndEndPoint(p)` `genRoute`'tan önce eklenmeli.

Eksik: (1) action goal alanı, (2) noktanın field ile **aynı CRS**'e dönüşümü,
(3) noktanın `generateRoute → plan → TspRouteMethod` boyunca taşınması, (4) per-cell stitch
davranışı, (5) BT portu.

### Koordinat/hassasiyet kararı (önemli)
Kullanıcı start noktasını field CRS'inde girer (GPS lat/lon ya da cartesian) — girdi için
`Coordinate` (`float32`) yeterli (poligonlarla aynı). Ama server GPS modda noktayı **UTM mutlak**
koordinata dönüştürür; UTM easting ~5e5 → `float32` cm hassasiyeti kaybeder. Bu yüzden
**dönüştürülmüş nokta `double` kalır ve mesaja geri yazılmaz**; C++ API boyunca
`std::optional<F2CPoint>` olarak taşınır.

### Değişecek Dosyalar
| Dosya | Değişiklik |
|---|---|
| `opennav_coverage_msgs/action/ComputeCoveragePath.action` | `use_start_pose` + `start_pose` goal alanları (girdi, field CRS) |
| `opennav_coverage/include/opennav_coverage/route_method.hpp` | `plan(...)`'a trailing `std::optional<F2CPoint>` |
| `opennav_coverage/src/route_method.cpp` | `TspRouteMethod`: `setStartAndEndPoint` uygula |
| `opennav_coverage/include/opennav_coverage/route_generator.hpp` | `generateRoute(...)`'a trailing `std::optional<F2CPoint>` |
| `opennav_coverage/src/route_generator.cpp` | noktayı `method->plan(...)`'a geçir |
| `opennav_coverage/src/coverage_server.cpp` | goal→F2CPoint + CRS transform (double) + `generateRoute`'a geçir |
| `opennav_coverage/include/opennav_coverage/utils.hpp` | GPS modda tek-nokta→UTM dönüşüm helper'ı |
| `opennav_coverage_bt/src/compute_coverage_path.{hpp,cpp}` | `use_start_pose` + `start_pose` portları |
| `opennav_coverage/test/test_route.cpp`, `test_server.cpp` | testler |

### Adım Adım

#### Adım 1 — Action goal'a başlangıç noktası (girdi, field CRS)
```
# Optional: fix the TSP route to start and return to this point (robot's current pose).
# Only used when generate_route=true and route resolves to TSP.
# Same coordinate system as `polygons` (cartesian or GPS per frame_id).
bool use_start_pose False
opennav_coverage_msgs/Coordinate start_pose   # axis1=x/lon, axis2=y/lat
```
> `Coordinate` mesajı zaten var (`float32 axis1`, `float32 axis2`).

#### Adım 2 — `plan(...)` / `generateRoute(...)` imzalarına opsiyonel nokta
`plan` polimorfik arayüz stateless kalmalı (`default_generator_` yeniden kullanıldığı için setter
riskli). Trailing opsiyonel argüman:
```cpp
// route_method.hpp — base + iki override
virtual F2CRoute plan(
  const F2CCells & cells, const F2CSwathsByCells & swaths_by_cells,
  const opennav_coverage_msgs::msg::RouteMode & settings,
  const std::optional<F2CPoint> & start_end = std::nullopt) = 0;
```
```cpp
// route_generator.hpp — aynı trailing arg; gövdede method->plan(cells, sbc, eff, start_end)
```
Varsayılan `std::nullopt` → mevcut çağrılar/testler değişmeden derlenir.

- **Non-TSP modda `use_start_pose=true` (net davranış):** `RouteGenerator::generateRoute`
  `action_type`'ı zaten çözüyor ve `logger_`'ı var → **tek net uyarı yeri burası.** Kural:
  `start_end.has_value() && action_type != RouteType::TSP` ise bir kez `RCLCPP_WARN`
  ("start_pose ignored: only used in TSP route mode") ver, sonra normal devam et (nokta
  yok sayılır, rota değişmez). Böylece davranış **sessiz değil**, belirli ve test edilebilir.
- **`SwathOrderMethod::plan`**: `start_end`'i **kullanmaz** — start/end TSP-özgü bir kavram;
  bu, aynı sınıfın `settings.tsp_*` knob'larını da kullanmamasıyla tutarlı (dead code değil,
  arayüz tekdüzeliği). Parametreyi `(void)start_end;` ile işaretle; uyarı yukarıda
  `generateRoute`'ta verildiği için burada dal ekleme.

#### Adım 3 — `TspRouteMethod::plan`'da uygula (per-cell stitch inceliği)
```cpp
if (total_swaths <= max_swaths_for_global_route_) {
  f2c::rp::RoutePlannerBase rp;
  if (start_end) { rp.setStartAndEndPoint(*start_end); }
  return rp.genRoute(cells, swaths_by_cells, false, d_tol, redirect_swaths, time_limit, search_for_optimum);
}
// Stitch fallback: birden fazla rp → start_end burada UYGULANMAZ
if (start_end) {
  RCLCPP_WARN(logger_,
    "start_pose ignored: swath count exceeds max_swaths_for_global_route; route is stitched per-cell.");
}
// ... mevcut stitch döngüsü aynen ...
```
> **Kapsam kararı:** B.9 **global-route yolunda** desteklenir. Stitch fallback'te (büyük/çok
> hücreli alan) tek bir başlangıç noktası anlamlı uygulanamaz (her hücrenin kendi `rp`'si var,
> hücreler indeks sırasıyla geziliyor); orada tek bir `RCLCPP_WARN` ile atlanır. Bu bir çalışma-
> zamanı dalı, dead code değil. Stitch yolunda start noktasını desteklemek (en-yakın-hücre
> sıralaması, seçenek B) → `F2C_V2_ERTELENEN_ISLER.md` §6.1; bu planın kapsamında kod eklenmez.

#### Adım 4 — Server'da goal→F2CPoint + CRS dönüşümü (⚠️ kritik)
`coverage_server.cpp`, `generateRoute` çağrısından önce:
```cpp
std::optional<F2CPoint> start_end;
if (goal->use_start_pose) {
  F2CPoint p(goal->start_pose.axis1, goal->start_pose.axis2);   // field CRS
  if (!cartesian_frame_) {
    p = util::transformPointToUTM(p, master_field.getCRS());     // double kalır; Adım 5
  }
  start_end = p;
}
F2CRoute route = route_gen_->generateRoute(route_cells, swaths_by_cells, goal->route_mode, start_end);
```
> **⚠️ En kritik risk:** `route_cells`/`swaths_by_cells` `field = master_field.getField()
> .getGeometry(0)` uzayında (UTM'e dönüştürülmüş MUTLAK, ref-point çıkarılmamış). Start noktası
> **birebir aynı uzayda** olmalı. GPS modda `transformToUTM` bir Field/geometri üzerinde çalışır,
> bare-Point'te değil.
>
> **İki dal ayrı ayrı test edilir:**
> - **Cartesian modda (`cartesian_frame_==true`):** `transformPointToUTM` **çağrılmaz**; nokta
>   girildiği koordinatlarla aynen taşınır (field da dönüştürülmediği için aynı uzayda). → T-B9-3.
> - **GPS modda (`cartesian_frame_==false`):** `transformPointToUTM` ile UTM'e taşınır. → T-B9-2.

#### Adım 5 — `utils.hpp::transformPointToUTM` helper'ı
Noktayı geçici `F2CField`/`F2CCell`'e sarıp `f2c::Transform::transformToUTM` uygula, sonucu tek
`F2CPoint` (double) olarak döndür. Yalnızca GPS modda çağrılır (dead değil). Sade ve test edilebilir.

#### Adım 6 — BT node portu
`compute_coverage_path.{hpp,cpp}`'ye B.1/B.3 port desenini izleyerek `use_start_pose` (bool) +
`start_pose` (nokta) portları; goal'a doldur. Nokta portu için mevcut BT dönüşüm desenini izle.

### B.9 Entegrasyon Sırası
1. `opennav_coverage_msgs` (yeni action alanları) → önce.
2. `opennav_coverage` (`route_method`, `route_generator`, `coverage_server`, `utils`).
3. `opennav_coverage_bt` (portlar).
4. `route_mode ≠ TSP` iken `use_start_pose=true` → `generateRoute` bir kez `RCLCPP_WARN` verir,
   nokta yok sayılır, rota değişmez (T-B9-5).

### B.9 Testler (otomatik — CI'da koşar)
- **T-B9-1 (unit, test_route — global, honored):** global yolda (cap altında küçük alan)
  `use_start_pose` verilen sabit noktayla `F2CRoute::startPoint()` ≈ nokta (d_tol içinde).
- **T-B9-2 (unit, CRS — GPS):** GPS modda `transformPointToUTM` çıktısının swath koordinat
  uzayıyla aynı origin/ölçekte olduğunu doğrula (Adım 4/5 GPS dalını kilitler).
- **T-B9-3 (unit, CRS — Cartesian):** cartesian modda `transformPointToUTM` **çağrılmaz**; nokta
  girildiği koordinatlarla aynen kullanılır ve `startPoint()` ona eşit (Adım 4 cartesian dalı).
- **T-B9-4 (unit, stitch):** swath sayısı cap'i aşan alanda `use_start_pose` → rota üretilir,
  start/end uygulanmaz, `RCLCPP_WARN` loglanır, çökme yok (Adım 3 davranışı).
- **T-B9-5 (unit, non-TSP):** `use_start_pose=true` + BOUSTROPHEDON (veya başka non-TSP) →
  `generateRoute` bir kez `RCLCPP_WARN` verir; üretilen rota `use_start_pose=false` ile
  **birebir aynı** (nokta yok sayıldı, davranış belirli — Adım 2 kararı).
- **T-B9-6 (unit, nullopt regresyon):** start noktası verilmeyince (`std::nullopt`) çıktı
  eski davranışla aynı (imza değişikliğinin regresyon kilidi).
- **T-B9-7 (integration, test_server — TSP, honored):** `use_start_pose=true` + TSP +
  `generate_path=true` → SUCCEEDED; `nav_path.poses.front()` başlangıç noktasına yakın.
- **T-B9-8 (integration, test_server — regresyon):** `use_start_pose=false` (aynı goal) →
  çıktı mevcut baseline ile aynı; açık regresyon (örtük değil).
- **T-B9-9 (BT):** yeni portlar blackboard'dan doğru okunur (B.1 TSP port testi deseni).

### B.9 El ile Doğrulama (CI dışı — otomatik assertion yok)
- **M-B9-1 (RViz davranışsal):** deterministik 50×50 poligon + köşe start noktası → rotanın o
  köşeden başlayıp döndüğü görsel teyit. Unit test **değildir**; CI'a girmez, elle koşulur
  (lifecycle configure+activate, `compute_coverage_path` goal). Otomatik testlerin
  yakalayamayacağı geometrik doğruluğu gözle kontrol için.

### Ertelenen İşler
- **Stitch yolunda start noktası (en-yakın-hücre sıralaması)** → `F2C_V2_ERTELENEN_ISLER.md` §6.1.
  Bu planın kapsamı dışında; kod eklenmez.

### Riskler / Yürütmede Doğrulanacak
- **CRS dönüşümü (en yüksek risk):** start noktası genRoute'a giren cell/swath'larla aynı MUTLAK
  UTM uzayında olmalı; GPS modda bare-Point `transformToUTM`'e giremez → sarma gerekir (T-B9-2).
- **float32 tuzağı:** dönüştürülmüş UTM noktasını mesaja geri yazma; `double` F2CPoint olarak
  C++ API'sinden taşı.
- **Geriye uyumluluk:** `plan`/`generateRoute` yeni argümanı `std::nullopt` default → mevcut
  refactor testleri (`test_route.cpp`) değişmeden geçmeli.
- Tüm build/test **lyrical/rolling devcontainer** içinde (host değil); F2C v2 `sudo ldconfig` ve
  `bt`/`navigator` kapsam notları migration durumundaki gibi geçerli.
