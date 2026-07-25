# F2C v2 — B.1: Çok-Hücreli TSP Rota Planlama (`RoutePlannerBase`) 🔴

> **Durum:** Planlandı — henüz uygulanmadı.
> **Ön koşul:** B.3 (decomposition) ✅ tamamlandı — çok-hücreli swath üretimi mevcut.
> **Bağımlılık:** Bu iş B.2'nin (F2CRoute → path/msg köprüsü) **minimal bir dilimini**
> zorunlu olarak içerir; aksi halde TSP çıktısı pipeline'da bir yere akamaz. Ayrıntı
> aşağıda "Mimari Karar" bölümünde.

---

## Nedir? (Ne Var, Ne Eksik)

**Şu an** — [route_generator.cpp:23-54](opennav_coverage/src/route_generator.cpp#L23-L54):
`RouteGenerator::generateRoute(Swaths, RouteMode) → Swaths`. Yalnızca **tek-hücre**
desen sıralayıcıları (`BoustrophedonOrder`/`SnakeOrder`/`SpiralOrder`/`CustomOrder`
→ `genSortedSwaths`) kullanır. Bunlar swath'leri sadece bir **sıraya** dizer; hücreler
arası veya headland üzerinden gerçek navigasyon maliyetini hesaba katmaz, dönüşleri
sonradan `PathPlanning` naif üretir.

**v2 yeniliği** — `f2c::rp::RoutePlannerBase::genRoute(...)` (repo'daki v2.0.0 kaynağıyla
teyit edildi, [route_planner_base.h:39-42](Fields2Cover/include/fields2cover/route_planning/route_planner_base.h#L39-L42)):

```cpp
virtual F2CRoute genRoute(
    const F2CCells& cells, const F2CSwathsByCells& swaths_by_cells,
    bool show_log = false, double d_tol = 1e-4, bool redirect_swaths = true,
    long int time_limit_seconds = 1, bool search_for_optimum = false);
```

- OR-Tools TSP tabanlı; **headland halkaları üzerinden geçen** bağlantılarla rota kurar.
- Çoklu hücre/poligon destekler (decomposition çıktısıyla doğal eşleşir).
- Çıktı `Swaths` değil **`F2CRoute`** — hem sıralı swath gruplarını hem aralarındaki
  bağlantı yollarını (`getConnections() → MultiPoint`) tutar.

`F2CRoute` API'si ([types/Route.h](Fields2Cover/include/fields2cover/types/Route.h)):
`getVectorSwaths()`, `getSwaths(i)`, `getConnections()`, `getConnection(i)`,
`asLineString()`, `length()`, `startPoint()/endPoint()`, `sizeVectorSwaths()`,
`sizeConnections()`, `isEmpty()`, `clone()`.

`F2CSwathsByCells` ([types/SwathsByCells.h](Fields2Cover/include/fields2cover/types/SwathsByCells.h)):
`operator[]`, `at(i)`, `size()`, `sizeTotal()`, `push_back(Swaths)`, `flatten() → Swaths`.

---

## Mimari Karar (kritik — plan buraya dayanıyor)

B.1 üç yerde mevcut pipeline'ın tip sözleşmesini kırar. Kararlar:

### K1 — Tip kırılması: TSP `F2CRoute` döndürür, `Swaths` değil
Mevcut `generateRoute(...) → Swaths` desen sıralayıcılar için **olduğu gibi korunur**
(geriye uyumluluk). TSP için **ayrı bir metot** eklenir:
`generateRouteTSP(cells, swaths_by_cells, settings) → F2CRoute`. Server, `route_mode`
`TSP` ise bu dalı, değilse eski dalı çağırır. (Varyant/`std::variant` yerine iki ayrı
metot — mevcut branch yapısıyla uyumlu, en az sürpriz.)

### K2 — Girdi: `swaths_by_cells` **düzleştirilmemiş** olmalı
[swath_generator.cpp:69-92](opennav_coverage/src/swath_generator.cpp#L69-L92)'deki
çok-hücreli overload sonucu `.flatten()` ile `Swaths`'e indirir — TSP'nin ihtiyacı olan
per-hücre yapıyı yok eder. Çözüm: `.flatten()` **öncesi** `F2CSwathsByCells` döndüren
**yeni, saf ekleme (additive)** bir metot çıkar (`generateSwathsByCells(...)`), **yalnız
TSP dalında** kullanılır. Mevcut `generateSwaths(...)` akışına **dokunulmaz** (çalışan kod
risk almaz). Altyapı F2C `generateBestSwaths(obj, width, cells)` zaten `F2CSwathsByCells`
döndürüyor.

> **NOT (ertelenen optimizasyon):** İleride mevcut düzleştiren `generateSwaths`'i
> `generateSwathsByCells(...).flatten()` üzerine sarıp swath üretimini tek yola toplamak
> mümkün — ama bu çalışan decomp/non-decomp yolunu değiştirir. **Süreç bitince ayrıca
> denenecek** (bkz. Adım 2/6 NOT'ları + Riskler). Bu PR'da TSP dalı swath'i ikinci kez
> üretir (yalnız TSP istenince); kabul edilen, dokümante edilmiş bir maliyet.

### K3 — Girdi: `cells` = rotanın "içinden geçtiği" headland halkası
Tutorial deseni ([tutorials/5_route_planning.cpp:29-37](Fields2Cover/tutorials/5_route_planning.cpp#L29-L37))
**iki** headland inset'i kullanır: sığ olan `mid_hl` (rota buradan geçer, `genRoute`'un
`cells` argümanı) + derin olan `no_hl` (swath'ler buradan üretilir).

- **İlk kesim (önerilen, minimal):** Mevcut tek headland'i (`cells_no_headland`) **hem**
  `cells` argümanı **hem** swath kaynağı olarak kullan. Bağlantılar swath-headland sınırı
  boyunca akar — geçerli, düşük riskli, mevcut tek-headland UX'iyle tutarlı. Tek-hücre
  durumunda tek `Field`'ı `F2CCells`'e sar (`addGeometry`).
- **İyileştirme (sonraki, opsiyonel):** Tutorial-sadık iki-headland: `cells` için ayrı,
  daha sığ bir "route connection headland" üret (ör. headland genişliğinin yarısı). Yeni
  bir `route_headland_width` parametresi gerektirir. Bağlantıların swath uçlarını kesme
  riskini azaltır. **Bu PR'da şart değil**; risk notu olarak bırak.

### K4 — Çıktı: `F2CRoute` → `Path` köprüsü (minimal B.2 dilimi)
F2C zaten sağlıyor:
`PathPlanning::planPath(robot, F2CRoute, TurningBase&) → F2CPath`
([path_planning.h:22](Fields2Cover/include/fields2cover/path_planning/path_planning.h#L22)).
Yani `generate_path=true` yolunda: `PathGenerator`'a `F2CRoute` alan bir overload ekle
→ `planPath(robot, route, curve)` çağır → **elimizde bir `Path` olur** → mevcut
`toCoveragePathMsg(Path,...)` / `toNavPathMsg(...)` **hiç değişmeden** çalışır (B.6/B.7
task_time, velocity dahil). Bu, B.1'i uçtan uca çalışır kılmak için gereken **tek**
B.2 parçasıdır.

### K5 — `generate_path=false` + TSP: **temiz reddet** (fallback yok)
TSP'nin asıl değeri swath'ler arası headland bağlantılarıdır; `generate_path=false` bunları
zaten üretmez, dolayısıyla anlamlı bir TSP çıktısı vermez. **Karar:** TSP `generate_path=true`
**şart koşar**; değilse temiz bir `CoverageException` fırlat (`INVALID_MODE_SET`, çökme yok).
Flatten fallback, `flattenRoute` yardımcısı ve `default_tsp_require_path` politikası
**eklenmez** (basitlik; bağlantısız TSP değersiz).

> **PR kapsamı özeti:** B.1 (TSP üretimi) + K4'ün minimal B.2 dilimi (`F2CRoute → Path`
> köprüsü) bu PR'da birlikte gider. Bağlantıları ayrı ayrı mesaja gömen / RViz'de çizen
> **tam B.2** ve tutorial-sadık iki-headland (K3 iyileştirmesi) sonraki iş olarak ayrılabilir.

---

## Etkilenen / Değişecek Dosyalar

| Dosya | Değişiklik |
|---|---|
| [types.hpp](opennav_coverage/include/opennav_coverage/types.hpp) | `RouteType::TSP = 5`; gerekirse `F2CRoute`/`F2CSwathsByCells` için typedef |
| [route_generator.hpp/.cpp](opennav_coverage/src/route_generator.cpp) | `generateRouteTSP(...)`, `toType/toString` TSP kolu, TSP parametre alanları + setter'lar |
| [swath_generator.hpp/.cpp](opennav_coverage/src/swath_generator.cpp) | `generateSwathsByCells(...) → F2CSwathsByCells` (flatten öncesi) çıkar |
| [path_generator.hpp/.cpp](opennav_coverage/src/path_generator.cpp) | `generatePath(const F2CRoute&, PathMode) → Path` overload'u (planPath route) |
| [coverage_server.cpp](opennav_coverage/src/coverage_server.cpp) | Rota dalını TSP için çatalla; `generate_path=false`+TSP → temiz throw (K5); YAML default + dynamic param |
| `RouteMode.msg` | `TSP` modu yorumu + TSP knob alanları (`tsp_redirect_swaths`/`tsp_time_limit`/`tsp_search_for_optimum`/`tsp_d_tol`) |
| [compute_complete_coverage_path.cpp/.hpp](opennav_coverage_bt/src/compute_complete_coverage_path.cpp) | **Ön koşul:** route-mode port'u bugün YOK → önce `route_mode_type` (string) port'u, sonra `tsp_*` port'ları (B.3 decomp-port commit'i 4d71264 şablon) |
| `test_route.cpp`, `test_server.cpp` | TSP birim + entegrasyon testleri |
| README / params doc | `TSP` modu + `tsp_*` parametre dökümantasyonu |

---

## Adım Adım (Entegrasyon)

### Adım 1 — `types.hpp`: enum + typedef
- `RouteType` enum'una `TSP = 5` ekle ([types.hpp:81-88](opennav_coverage/include/opennav_coverage/types.hpp#L81-L88)).
- Gerekirse okunabilirlik için `typedef F2CRoute Route;` ve `typedef F2CSwathsByCells SwathsByCells;` ekle (F2C makroları zaten global; şart değil).

### Adım 2 — `swath_generator`: düzleştirilmemiş çıktı (saf ekleme)
- `swath_generator.hpp`'ye public `F2CSwathsByCells generateSwathsByCells(const F2CCells&, const SwathMode&)` ekle.
- Gövde: `resolveSwathParams` + `generateBestSwaths(obj, width, cells)` / `generateSwaths(angle, width, cells)`
  (F2C bunları zaten `F2CSwathsByCells` döndürür) — **`.flatten()` çağırma**.
- **Mevcut `generateSwaths(...)` metotlarına DOKUNMA.** Bu yeni metot yalnız TSP dalında
  kullanılır; çalışan decomp/non-decomp swath yolu birebir korunur.
- ⚠️ Tek-hücre: `generateSwathsByCells` 1-hücreli girdide **tek elemanlı** `F2CSwathsByCells`
  döndürmeli (F2C cells-overload'u bunu doğal yapar; mevcut `Field`-kısayoluna gerek yok).

> **NOT (K2):** İleride `generateSwaths(F2CCells)`'i `generateSwathsByCells(...).flatten()`
> üzerine sarmak swath üretimini tekilleştirir; **süreç bitince denenecek** ertelenen
> optimizasyon. Şimdilik TSP istenince swath ikinci kez üretilir (kabul edilen maliyet).

### Adım 3 — `route_generator`: TSP üretici
- `route_generator.hpp`'ye ekle:
  ```cpp
  F2CRoute generateRouteTSP(
    const F2CCells & cells,
    const F2CSwathsByCells & swaths_by_cells,
    const opennav_coverage_msgs::msg::RouteMode & settings);
  ```
- `.cpp` gövdesi:
  ```cpp
  f2c::rp::RoutePlannerBase rp;
  return rp.genRoute(
    cells, swaths_by_cells,
    /*show_log=*/false,
    /*d_tol=*/settings.tsp_d_tol,   // varsayılan 1e-4 = bilerek F2C default; YAML/msg ile ayarlanabilir
    /*redirect_swaths=*/settings.tsp_redirect_swaths,
    /*time_limit_seconds=*/settings.tsp_time_limit,
    /*search_for_optimum=*/settings.tsp_search_for_optimum);
  ```
- `toType()`/`toString()`'e `"TSP"` kolu ekle. TSP knob'ları için default alanları +
  setter'lar (`default_tsp_*`), mevcut `default_spiral_n_`/`default_custom_order_` desenini izle.
- `createGenerator()` **değişmez** (TSP `SingleCellSwathsOrderBase` değil; ayrı yol).

### Adım 4 — `path_generator`: F2CRoute overload'u
- `generatePath(const Swaths&, ...)`'in yanına ekle:
  ```cpp
  Path PathGenerator::generatePath(
    const F2CRoute & route, const opennav_coverage_msgs::msg::PathMode & settings)
  {
    // ... eğri seçimi mevcut generatePath ile aynı (ortak yardımcıya çıkarılabilir) ...
    curve->setDiscretization(turn_point_distance);
    return generator_->planPath(robot_params_->getRobot(), route, *curve);
  }
  ```
- Eğri seçim bloğu ([path_generator.cpp:26-49](opennav_coverage/src/path_generator.cpp#L26-L49))
  iki overload'da tekrar etmesin diye `resolveCurve(settings) → (curve, turn_point_distance)`
  private yardımcısına çıkar.

### Adım 5 — ~~`utils.hpp`: `flattenRoute`~~ **(KALDIRILDI — K5 basitleştirmesi)**
K5 artık `generate_path=false`+TSP'yi temiz reddediyor; flatten fallback ve `flattenRoute`
yardımcısı gerekmiyor. `utils.hpp` **değişmez**. (Adım numaraları sabit tutuldu.)

### Adım 6 — `coverage_server.cpp`: rota dalını TSP için çatalla (mevcut swath akışına dokunma)

**Kural:** Mevcut swath üretimi ([coverage_server.cpp:200-223](opennav_coverage/src/coverage_server.cpp#L200-L223))
**birebir korunur** — `swaths` (düz `Swaths`) eskisi gibi üretilir; görselleştirme ve non-TSP
yolları hiç değişmez. TSP dalı **saf ekleme**: yalnız TSP istenince alanı `F2CCells`'e sarıp
`generateSwathsByCells(...)` ile düzleştirilmemiş `sbc`'yi üretir.

```cpp
if (goal->generate_route) {
  const bool is_tsp = (route_gen_->resolveType(goal->route_mode) == RouteType::TSP);
  if (is_tsp) {
    // (a) Alanı F2CCells'e getir (decomp: çok hücre; değilse tek Field'ı sar)
    F2CCells field_cells;
    if (do_decomp) {
      field_cells = cells_no_headland;              // zaten F2CCells (B.3)
    } else {
      field_cells.addGeometry(field_no_headland);   // tek Field -> F2CCells
    }
    // (b) TSP için düzleştirilmemiş swath (Adım 2). NOT: swath burada 2. kez üretilir
    //     (yalnız TSP istenince) — ertelenen tekilleştirme optimizasyonu, bkz. Adım 2/Riskler.
    F2CSwathsByCells sbc = swath_gen_->generateSwathsByCells(field_cells, goal->swath_mode);

    F2CRoute route =
      route_gen_->generateRouteTSP(field_cells, sbc, goal->route_mode);   // K3: cells = field_cells
    if (route.isEmpty()) { throw CoverageException("TSP route planner returned empty route"); }

    if (!goal->generate_path) {
      // K5: bağlantısız TSP anlamlı çıktı vermez -> temiz reddet (fallback yok)
      throw CoverageException("TSP route requires generate_path=true");
    }
    path = path_gen_->generatePath(route, goal->path_mode);   // K4
    // ... buradan itibaren mevcut Path yolu AYNEN (toCoveragePathMsg(Path,...),
    //     toNavPathMsg + velocities/is_backward, task_time) ...
  } else {
    // MEVCUT desen-sıralayıcı yolu, değişmeden
    Swaths route = route_gen_->generateRoute(swaths, goal->route_mode);
    // ... mevcut generate_path true/false dalları aynen ...
  }
}
```

- **`generate_path=false` + TSP:** temiz `CoverageException` → `INVALID_MODE_SET` (K5; çökme
  yok). Fallback/politika parametresi yok.
- **Görselleştirme:** `visualizer_->visualize(...)` mevcut `swaths`'i alır
  ([coverage_server.cpp:261-264](opennav_coverage/src/coverage_server.cpp#L261-L264)) — değişmez;
  TSP'de bağlantılar `path` üzerinden görünür.
- `route_gen_->resolveType(RouteMode)` — server hangi dalı çağıracağını bilsin diye `toType`'ı
  public ince sarmalayıcıyla dışa aç (tip çözme mantığı RouteGenerator'da tek yerde kalır).

### Adım 7 — `RouteMode.msg` + action
- `RouteMode.msg`'e ekle (ROS msg bool default'u **küçük harf**: `true`/`false`):
  ```
  # mode yorumuna TSP ekle: BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM, TSP
  bool     tsp_redirect_swaths    true    # genRoute redirect_swaths
  uint16   tsp_time_limit         1       # genRoute time_limit_seconds
  bool     tsp_search_for_optimum false   # genRoute guided local search
  float64  tsp_d_tol              0.0001  # genRoute d_tol — 1e-4 = bilerek F2C default
  ```
- `ComputeCoveragePath.action` değişmez (`route_mode` zaten `RouteMode` içeriyor).
- **Not:** `tsp_*` alanları per-request (RouteMode) knob'ları; server-geneli aynılarının
  YAML default'ları Adım 8'de (`default_tsp_*`).

### Adım 8 — YAML default + dynamic params
- `RouteGenerator` ctor'unda `default_route_type` zaten var; `TSP` string'i artık geçerli.
  `default_tsp_redirect_swaths` / `default_tsp_time_limit` / `default_tsp_search_for_optimum` /
  `default_tsp_d_tol` parametrelerini `declare_parameter_if_not_declared` ile ekle.
- **`default_tsp_d_tol` (double, default `1e-4`):** genRoute `d_tol` — **bilerek F2C default'u**;
  nadiren ayarlanır ama hard-code kalmasın diye YAML'e açılır (+ README).
- [coverage_server.cpp:305-331](opennav_coverage/src/coverage_server.cpp#L305-L331) dynamic
  param callback'ine yeni `default_tsp_*` kollarını ekle (mevcut `default_route_type`/
  `default_spiral_n` desenini izle; `default_tsp_d_tol` → `PARAMETER_DOUBLE` kolu).

### Adım 9 — BT node port'ları
- ⚠️ **Ön koşul (doğrulandı):** BT node'da bugün route-mode port'u **yok** — yalnız
  `decomp_mode_type`/`decomp_split_angle` + `generate_route`/`generate_path` bool'ları var
  ([compute_complete_coverage_path.hpp:92-98](opennav_coverage_bt/include/opennav_coverage_bt/compute_complete_coverage_path.hpp#L92-L98));
  route modu şu an sadece YAML `default_route_type`'tan geliyor. TSP'yi BT'den seçebilmek için
  **önce** `route_mode_type` (string, default `"UNKNOWN"`) port'u + `getInput(...) → goal_.route_mode.mode`
  ekle.
- Sonra yeni `tsp_*` port'larını (`tsp_redirect_swaths`/`tsp_time_limit`/`tsp_search_for_optimum`/`tsp_d_tol`)
  ekle — **B.3'ün "add BT node ports for decomposition control" commit'i (4d71264) birebir
  şablon**. XML örnek/doxygen açıklamalarını da güncelle.

### Adım 10 — Dökümantasyon
- README + parametre tablosuna `TSP` modu ve `tsp_*` parametrelerini işle.
- `F2C_V2_GUNCELLEME_VE_YENI_OZELLIKLER.md`'de B.1 satırını "✅ tamamlandı" + gerçek
  entegrasyon notlarıyla güncelle (K3 minimal yaklaşımı, iki-headland ertelemesi vs.).

---

## Test Stratejisi

> ⚠️ **Determinizm uyarısı:** `genRoute` OR-Tools TSP kullanır; `time_limit_seconds` +
> arama moduna bağlı olarak **birebir sıralama değişebilir**. Testler **kesin sıraya değil,
> yapısal değişmezlere** assert etmeli. Tekrarlanabilirlik için `tsp_search_for_optimum=false`
> + sabit `tsp_time_limit` kullan.

### Birim — `test_route.cpp`
- **B1-T1 — TSP boş-olmayan rota:** Çok-swath'li deterministik `F2CCells` + `F2CSwathsByCells`
  üret; `generateRouteTSP(...)` çağır. Assert: `!route.isEmpty()`,
  `route.sizeVectorSwaths() >= 1`, `route.length() > 0`.
- **B1-T2 — Kapsama korunur:** Rotadaki toplam swath sayısı (`Σ getVectorSwaths()[i].size()`)
  girdi `swaths_by_cells.sizeTotal()`'a eşit — TSP swath düşürmemeli (redirect_swaths=true
  ile yön değişebilir, sayı değişmez).
- **B1-T3 — Bağlantı sayısı tutarlı:** `sizeConnections()` swath grupları arası geçişlerle
  tutarlı (grup sayısı − 1 civarı). Kesin değer yerine `>= 0` + `asLineString().size() > 0`.
- **B1-T4 — Tek hücre:** `F2CSwathsByCells` tek girdiyle de çökmeden rota döndürmeli.
- **B1-T11 — `toType`/`toString` TSP kolu:** `TEST(RouteTests, TestrouteUtils)` içindeki
  `RouteShim` desenine ekle ([test_route.cpp:60-85](opennav_coverage/test/test_route.cpp#L60-L85)):
  `toTypeShim("TSP")==RouteType::TSP`, `toTypeShim("tsp")==RouteType::TSP`,
  `toStringShim(RouteType::TSP)=="TSP"`.
  ⚠️ **İncelik (K1):** TSP `createGenerator`'dan geçmez (ayrı yol; `SingleCellSwathsOrderBase`
  değil) → `createGeneratorShim(RouteType::TSP)` **nullptr** dönmeli:
  `EXPECT_FALSE(generator.createGeneratorShim(RouteType::TSP))`.

### Birim — `test_swath.cpp`
- **B1-T5 — `generateSwathsByCells` düzleştirmez:** Çok-hücreli girdide dönen
  `F2CSwathsByCells.size()` hücre sayısıyla eşleşir; `flatten().size()` mevcut
  `generateSwaths(F2CCells)` çıktısıyla aynı (yeni metot eski düz çıktıyla tutarlı —
  tekilleştirme ertelense de eşdeğerlik burada kanıtlanır).

### Birim — `test_path.cpp`
- **B1-T6 — `generatePath(F2CRoute)`:** Küçük bir `F2CRoute` için overload bir `Path`
  döndürür, `path.size() > 0`, `getTaskTime()` finite (B.7 ile uyum).

### Entegrasyon — `test_server.cpp`
- **B1-T7 — TSP uçtan uca:** Deterministik 50×50 poligon goal'u, `route_mode.mode="TSP"`,
  `generate_route=true`, `generate_path=true`. Assert: `SUCCEEDED`, `coverage_path` boş değil,
  `contains_turns==true`, `task_time` finite ve > 0, `nav_path.poses` boş değil.
- **B1-T8 — TSP + decomposition:** L-şekilli poligon + `generate_decomp=true` + `route_mode=TSP`.
  Assert: `SUCCEEDED`, çok-hücre swath'leri kapsanır (swath sayısı non-decomp'tan az —
  B.3 ile aynı L-tarla fixture'ı kullan).
- **B1-T9 — `generate_path=false` + TSP → temiz reddet:** `route_mode="TSP"`,
  `generate_path=false`. Assert: `INVALID_MODE_SET` error code, çökme yok (K5 — fallback yok).
- **B1-T10 — Boş/dejenere alan:** Tek swath'e düşen küçük alan → ya geçerli rota ya da temiz
  `CoverageException` (çökme yok, `INTERNAL_F2C_ERROR` değil ideal).
- **B1-T13 — `default_tsp_*` dynamic params:** `TEST(ServerTest, testDynamicParams)`
  ([test_server.cpp:217-254](opennav_coverage/test/test_server.cpp#L217-L254)) desenine ekle:
  `set_parameters_atomically` ile `default_tsp_redirect_swaths`/`default_tsp_time_limit`/
  `default_tsp_search_for_optimum`/`default_tsp_d_tol` set et, `get_parameter(...)` ile
  geri oku ve assert et (mevcut `default_route_type`/`default_spiral_n`/`default_custom_order`
  kollarının yanına). Parametrelerin doğru declare + dynamic-callback'e bağlı olduğunu kanıtlar.

### Birim/Entegrasyon — `test_compute_coverage_path.cpp` (opennav_coverage_bt)
- **B1-T14 — BT TSP port'ları:** `TEST_F(ComputeCoveragePathActionTestFixture, test_tsp_ports)`
  — **`test_decomp_ports` birebir şablon** ([test_compute_coverage_path.cpp:163-189](opennav_coverage_bt/test/test_compute_coverage_path.cpp#L163-L189)).
  ⚠️ **Ön koşul (Adım 9):** BT node'da route-mode port'u bugün yok → önce `route_mode_type`
  (string) port'unu ekle. BT XML'de `route_mode_type="TSP"` +
  `tsp_redirect_swaths`/`tsp_time_limit`/`tsp_search_for_optimum`/`tsp_d_tol` attribute'larını
  ver, tree'yi tick'le, blackboard'dan `goal`'ü çek; `goal.route_mode.mode=="TSP"` ve `tsp_*`
  alanları beklenen değerlerde. Port'ların XML → goal akışını kanıtlar.

### Davranışsal — RViz (unit'in yakalayamadığı)
Migration'daki aynı kurulum (coverage_server lifecycle configure+activate; kapalı-halka
poligon; `static_transform_publisher world map`):
- **B1-V1:** 50×50 poligon, `TSP` → rota (swath'ler + headland üzerinden bağlantı dönüşleri)
  doğru render. Aynı tarlada `BOUSTROPHEDON` ile görsel kıyas — TSP daha kısa/akıllı bağlantılar
  vermeli.
- **B1-V2:** L-şekilli tarla + decomp + TSP → hücreler arası mantıklı geçişli kapsama.
- **Kontrol:** bağlantıların swath uçlarını aşırı kesip kesmediğine bak (K3 minimal yaklaşımının
  bilinen riski — kesiyorsa iki-headland iyileştirmesi gerekçesi belgelenir).

### Genel akış
```
# devcontainer içinde (rolling-full veya lyrical-full)
colcon build --packages-select opennav_coverage_msgs opennav_coverage opennav_coverage_bt
colcon test  --packages-select opennav_coverage opennav_coverage_bt   # bt: B1-T14
colcon test-result --verbose
# + RViz davranışsal kontrol (B1-V1/V2)
```

---

## Riskler / Dikkat Edilecekler

- **OR-Tools nondeterminizmi:** Testler yapısal değişmezlere assert etmeli (bkz. determinizm
  uyarısı). CI'da `tsp_time_limit` küçük tut (1 sn) ama flaky olursa `search_for_optimum=false`
  ile sabitlenmiş olması yardımcı.
- **K3 minimal yaklaşım:** `cells` argümanı swath-headland ile aynı → bağlantılar swath sınırı
  boyunca akar, uçları kesebilir. Kabul edilebilir ilk kesim; kesme gözlenirse tutorial-sadık
  iki-headland iyileştirmesine geç.
- **Swath çift-üretim (Adım 6, kabul edilen):** TSP istenince swath iki kez üretilir — mevcut
  düz `swaths` + TSP için `sbc`. Mevcut swath akışına dokunulmadığı için non-TSP yollarda
  regresyon riski **YOK**. Çift-üretim yalnız TSP dalında ve pahalı açı taraması tekrarı demek;
  **ertelenen optimizasyon:** süreç bitince `generateSwaths`'i `generateSwathsByCells().flatten()`
  üzerine sarıp tekilleştirmeyi dene (bkz. Adım 2 NOT). Denerken B1-T5 ile düz çıktının eskisiyle
  aynı kaldığını kanıtla.
- **Link bağımlılığı:** `planPath(robot, route, curve)` ve `RoutePlannerBase` OR-Tools'a bağlı;
  F2C hedefi bunu zaten taşıyor (mevcut `planPath(swaths)` linkleniyor). **Yeni link deps
  beklenmiyor** — yine de ilk build'de `libortools` sembol hatası olursa
  [[f2c-v2-migration]]'daki `ldconfig` gotcha'sını hatırla (devcontainer).
- **UTM/CRS:** TSP `generate_path=true` şart koştuğu için dönüşüm her zaman `Path` üzerinden
  mevcut util'lerle olur (değişmez); ayrı bir `Swaths` mesaj yolu yok.
- **`resolveType` sızıntısı:** Server'ın TSP olup olmadığını bilmesi için tip çözme mantığı
  RouteGenerator'da tek yerde kalmalı (kopyalama yok) — `toType`'ı public ince sarmalayıcıyla aç.
- **Geriye uyumluluk:** `route_mode` boş/UNKNOWN veya eski modlar → **davranış birebir aynı**
  kalmalı (regresyon testleri mevcut `test_route.cpp`/`test_server.cpp` yeşil kalarak korunur).

---

## Uygulama Sırası (commit'leme)

B.3-B.7'deki gibi her madde ayrı commit (`B.1: ...`):

1. `types.hpp` — `RouteType::TSP`.
2. `swath_generator` — `generateSwathsByCells` **saf ekleme** (mevcut akışa dokunma) (+ B1-T5).
3. `route_generator` — `generateRouteTSP` (d_tol dahil) + toType/toString + `default_tsp_*` (+ B1-T1..T4).
4. `path_generator` — `generatePath(F2CRoute)` overload + `resolveCurve` refactor (+ B1-T6).
5. `coverage_server` — rota dalı çatallama (TSP + `generate_path=false`→throw) + YAML/dynamic param (+ B1-T7..T10, T13).
6. `RouteMode.msg` (`tsp_*` + `tsp_d_tol`) + BT `route_mode_type`/`tsp_*` port'ları (Adım 7, 9) (+ B1-T14).
7. README/doc + `F2C_V2_GUNCELLEME...` güncellemesi.

**B.9 — AYRI iş (bu PR'a DAHİL DEĞİL):** `RoutePlannerBase::setStartAndEndPoint(F2CPoint)`
kullanıcı kararıyla ayrı bir PR/branch'te ele alınacak ve **testi ayrı yapılacak**. Bu plan
B.1'i B.9'suz uçtan uca tamamlar; `generateRouteTSP` ileride start/end alacak şekilde
genişletilebilir ama şimdilik eklenmez.

**Sonraki iş (bu PR dışı):** Tam B.2 — `utils.hpp`'ye `F2CRoute` overload'ları
(`getConnections()`'ı ayrı ayrı coverage msg'e göm + RViz'de bağlantı çizimi) ve K3
tutorial-sadık iki-headland iyileştirmesi.
