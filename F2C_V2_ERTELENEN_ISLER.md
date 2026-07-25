# F2C v2 — Ertelenen / Sonraya Bırakılan İşler

> Bu dosya, uygulama planlarında ("sonraya bırakıldı", "ertelenen optimizasyon", "bu PR
> kapsamı dışı", "opsiyonel", "atlandı" olarak işaretlenen) ve **hâlâ tamamlanmamış** olan
> maddeleri tek yerde toplar. Çözülen/kapanan maddeler listeden çıkarılır (bkz. commit
> geçmişi doğrulaması).
> **Kaynaklar:** B.1, B.3+B.5, B.4, B.5, B.9 planları. **B.2 / B.8 (facade) hariç tutuldu.**
> B.6+B.7 planındaki tek ertelenen madde (`reduce()` adımı) 2026-07-10'da uygulandı, bu
> yüzden o plan artık burada listelenmiyor.
>
> Her madde: *ne ertelendi · gerekçe · nereden geldiği (plan + karar/adım) · tetikleyici*.

---

## 1. B.1 — Çok-Hücreli TSP Rota Planlama

*(Kaynak: `F2C_V2_B1_UYGULAMA_PLANI.md`)*

> **NOT (2026-07-10):** B.1 planı "henüz uygulanmadı" der ama **aslında uygulandı ve genişletildi**:
> PR **#117** (`Add TSP route ordering`, çekirdek) + PR **#119** (`Route decomposed fields along
> headlands`, headland-first + per-cell TSP stitch) + #119'a katlanan **RouteMethod refactoru**.
> Planın swath çift-üretimi, K5 (bağlantısız route reddi) ve decompose→headland sırası gibi
> maddeleri bu süreçte **çözüldü** ve aşağıdan çıkarıldı. Kalanlar gerçekten açık. Bkz.
> [[headland-first-decomp-routing]], [[f2c-genroute-badalloc-percell-tsp]].

### 1.1 Tutorial-sadık iki-headland (K3 iyileştirmesi) — ✅ UYGULANDI (2026-07-11, farklı tasarımla)
- **Ne (plan):** `genRoute`'a ayrı sığ "route connection headland" (`route_headland_width`).
- **Uygulanan:** Goal alanı `route_headland_width` (0 = operation width) geldi ama koridor
  **yalnız fiilen paylaşılan hücre kenarlarında** oyuluyor (`generateHeadlandsBetweenCells`,
  kenar-tabanlı çizgi-buffer + alan filtresi). #119'un per-sub-cell ikinci tam inset'i (double
  headland) TAMAMEN kaldırıldı. Rota artık `(route_cells, swath_cells, sbc)` üçlüsüyle planlanıyor:
  per-cell TSP swath_cells'ten, hücreler-arası köprüler bitişik travel-hücre çiftinin sınır
  grafiğinden (`createShortestGraph`+`shortestPath`, kademeli fallback).
- **Tek-hücre (non-decomp):** tek genRoute, davranış değişmedi — o alt madde artık geçerli değil
  (bağlantı kesmeleri path katmanındaki sapma-oranı kuralıyla çözüldü).

### 1.2 Tam B.2 — F2CRoute bağlantılarını mesaja gömme + RViz çizimi — 🟡 AÇIK
- **Ne:** `getConnections()`'ı ayrı ayrı coverage msg'e gömmek + RViz'de bağlantı yollarını çizmek.
- **Gerçek durum (HEAD):** `toCoveragePathMsg(F2CRoute, ...)` overload'u eklendi
  (`utils.hpp:121`) ama yalnız `getVectorSwaths()`'i sıralı swath'e düzleştiriyor — **`getConnections()`
  ayrı gömülmüyor**. RViz hâlâ `swaths` + `path` çiziyor (`coverage_server.cpp:267-270`); bağlantılar
  yalnız `generate_path=true` yolunda `path` içindeki turn'ler olarak görünüyor. Ayrı bağlantı
  gömme + RViz çizimi **hâlâ yapılmadı**.
- **Yer:** Mimari Karar K4/K5 PR-kapsamı özeti + "Sonraki iş (bu PR dışı)".

---

## 2. B.3 Decomposition + B.5 HL_SWATH

*(Kaynak: `F2C_V2_B3_B5_UYGULAMA_PLANI.md`)*

### 2.1 Decomposed hücrelerin ayrı görselleştirmesi — TAŞINDI (deferred)
- **Ne:** Decomposed alt-hücreleri ayrı `MarkerArray` ile RViz'de göstermek (frame_id düzeltilmiş).
- **Gerekçe:** Bu PR'da visualizer'a hiç dokunulmadı; `field_no_headland == field` olduğundan
  mevcut görselleştirme çalışıyor. Ayrı hücre görseli kapsam dışı.
- **Yer:** Adım 11 (PR'dan çıkarıldı) + Commit Yapısı notu → `F2C_V2_GUNCELLEME_VE_YENI_OZELLIKLER.md` → B.3'e taşındı.

### 2.2 Objektif tabanlı decomposition (`DecompObjective`) — KAPSAM DIŞI
- **Ne:** `decompose(cells, obj)`'un ikinci parametresi olan `obj::DecompObjective`'i UI'dan ayarlamak.
- **Gerekçe:** Bu PR default'u (default-construct) kullanır. Bilinçli olarak kapsam dışı.
- **Yer:** Dikkat Edilecekler ("DecompObjective kullanılmıyor").
- **Tetikleyici:** Gerekirse `DecompMode.msg`'a ileride alan eklenerek açılır.

### 2.3 B.5 HL_SWATH densification — ✅ UYGULANDI (2026-07-10)
- **Ne:** `discretizeSwath()` yalnızca `SWATH` böler; `HL_SWATH` tek nokta olarak geçiyordu.
- **Çözüm:** `utils.hpp`'de `discretizeSwathLike()` yardımcısı eklendi (SWATH + HL_SWATH böler);
  `toNavPathMsg` artık bunu kullanıyor. Headland perimetre geçişi kontrolcüye yoğun path olarak gidiyor.
- **Test:** `TesttoNavPathMsgHLSwathDensified`.
- **Yer:** B.5 Kapsam + Dikkat Edilecekler. **§3** (B.5 planı) ile örtüşür.

### 2.4 `split_angle` fonksiyonel testi — BİLİNÇLİ ATLANDI
- **Ne:** Farklı split açılarının farklı decomp sonucu verdiğini doğrulayan test.
- **Gerekçe:** F2C iç davranışını test etmek olur; proje felsefesiyle çelişir. Setter throw
  etmiyor testlendi, yeterli.
- **Yer:** Dikkat Edilecekler.

---

## 3. B.5 — HL_SWATH Tipi (kapsam-dışı maddeler)

*(Kaynak: `F2C_V2_B5_UYGULAMA_PLANI.md` → "B.5 KAPSAMI DIŞI")*

- **3.1 Headland perimetrini otomatik path'e ekleme** — ✅ UYGULANDI (2026-07-10).
  `util::toHeadlandPerimeterPath(field_no_headland, cruise_vel)` tek kapalı HL_SWATH loop üretir;
  `coverage_server` bunu path'in başına/sonuna ekler. Test: `TesttoHeadlandPerimeterPath`.
- **3.2 Coverage server'da "headland'i de sür" modu** — ✅ UYGULANDI (2026-07-10).
  Goal alanları: `generate_headland_swaths` (bool, opsiyonel) + `headland_first` (bool, sıra).
  YAML: `default_generate_headland_swaths`. Ring sayısı üretmiyoruz — mevcut headland bandının
  dış halkasını tek loop olarak sürüyoruz (kullanıcı kararı).
- **3.3 HL_SWATH densification** — §2.3 ile aynı; ✅ uygulandı.

---

## 4. B.4 — NSwathModified

*(Kaynak: `F2C_V2_B4_UYGULAMA_PLANI.md`)*

### 4.1 Parametre dökümantasyonu güncellemesi — OPSİYONEL
- **Ne:** `coverage_server.hpp`'daki `default_swath_type` açıklamasına `NUMBER_MODIFIED` eklemek.
- **Gerekçe:** Adım 6 "opsiyonel" işaretli; işlevsel değil.
- **Yer:** Adım 6.

> B.4'te başka anlamlı ertelenen madde yok (küçük, bağımsız iş).

---

## 5. B.9 — Başlangıç/Bitiş Noktası Sabitleme

*(Kaynak: `F2C_V2_B9_UYGULAMA_PLANI.md`)*

### 5.1 Stitch yolunda start noktası (en-yakın-hücre sıralaması) — ✅ UYGULANDI (2026-07-11)
- **Uygulanan:** Çok-hücre TSP artık HER ZAMAN per-cell stitch (global genRoute kaldırıldı,
  `max_swaths_for_global_route` emekli). Hücreler en-yakın-komşu + yön-uyum cezasıyla sıralanıyor;
  her hücreye rotasının yakın ucundan giriliyor (`reversedRoute` ile ters çevirme). `start_pose`
  verilirse ilk hücre ona en yakın olan seçiliyor (WARN: tam nokta değil, en yakın hücre; tam nokta
  yalnız tek-hücre yolunda). Dikişler bitişik travel-hücre çifti grafiğinden yürüyor.
- **Bilinen kabul edilmiş kalıntılar:** koridor/çentik uçlarındaki mikro-swath kırpıntıları
  "dönüş+kırpıntı+dönüş" küçük artefaktları üretir (min_swath_length filtresi kapsama garantisi
  için REDDEDİLDİ); çentik tepesindeki köprü polyline'ında mini merdiven olabilir.

---

## Özet Tablo

> **Durum lejantı:** 🟡 açık · 🔄 kısmen açık · ⏸️ ayrı iş · ⬜ hâlâ açık (kabul edilmiş kısıt / opsiyonel).

| # | İş | Kaynak plan | Durum | Tetikleyici |
|---|---|---|---|---|
| 1.1 | İki-headland (route_headland_width) | B.1 | ✅ Uygulandı (2026-07-11, kenar-tabanlı koridor) | — |
| 1.2 | Tam B.2 (bağlantı msg + RViz) | B.1 | 🟡 Route→msg overload var; bağlantı gömme açık (RViz debug eklendi→istenerek kaldırıldı) | — |
| 2.1 | Decomposed hücre görselleştirme | B.3 | ⬜ Taşındı (GUNCELLEME→B.3) | — |
| 2.2 | DecompObjective ayarı | B.3 | ⬜ Kapsam dışı | DecompMode.msg alanı eklenirse |
| 2.3 | HL_SWATH densification | B.3/B.5 | ✅ Uygulandı (2026-07-10) | — |
| 2.4 | split_angle fonksiyonel testi | B.3 | ⬜ Bilinçli atlandı | — |
| 3.1 | Headland perimetri path'e ekleme | B.5 | ✅ Uygulandı (2026-07-10) | — |
| 3.2 | "Headland'i de sür" modu | B.5 | ✅ Uygulandı (2026-07-10) | — |
| 4.1 | default_swath_type doc | B.4 | ⬜ Opsiyonel | — |
| 5.1 | Stitch yolunda start (en-yakın-hücre sıralaması) | B.9 | ✅ Uygulandı (2026-07-11) | — |

### Gerçekten hâlâ açık olan başlıca işler
1. **Bağlantıların ayrı mesaja gömülmesi** (§1.2) — B.2'nin msg yarısı (RViz yarısı denendi, istenerek kaldırıldı).
2. **Mikro-swath kırpıntı artefaktları** (§5.1 notu) — min_swath_length filtresi kapsama garantisi
   nedeniyle reddedildi; ileride "koşullu güvenli filtre" (şerit örtüşme doğrulamalı) düşünülebilir.
