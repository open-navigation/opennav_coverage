# F2C v2 — Bölüm B.8 Uygulama Planı (Facade — opsiyonel)

> B.9'dan ayrıldı (`F2C_V2_B9_UYGULAMA_PLANI.md`). **Opsiyonel** iş kalemi.
> Kod satır/imza referansları `Feature/umutc/f2c-enhancements-main` HEAD'ine göredir.

---

## B.8 — Facade (`planCovPath` / `planCovRoute`) — Basit Mod 🟠

### Durum
`fields2cover.h` facade'i mevcut: `f2c::planCovPath(robot, field, Options, local_crs=true)` /
`planCovRoute(...)`. `struct Options{ hg_alg, hg_swaths, sg_alg, sg_obj, sg_angle, rp_alg,
pp_alg, decomp_alg, decomp_angle }`. `local_crs` bayrağı `coordinates_in_cartesian_frame`'e
birebir uyar.

**⚠️ Kısıt (repodaki `Fields2Cover/` kaynağından teyit edildi):** `Options::decomp_alg`
yalnızca `DecompAlg::NONE` (TRAPEZOIDAL/BOUSTROPHEDON header'da yorumlanmış) → **facade
decomposition YAPAMAZ.** Ayrıca özel sıralama, TSP knob'ları, `max_swaths_for_global_route`
cap'i, headland-first routing de facade'de yok.

### Karar Noktası (kod yazmadan önce)
Granüler hat (B.1–B.7) refactor sonrası tüm modları tek `generateRoute → F2CRoute` yolundan,
daha fazla kontrolle üretiyor. B.8 mimariyi genişletmez, yalnızca "tek çağrılık basit yol"
sunar ve gelişmiş özellikleri (decomp/TSP-cap) **desteklemez**.

**Öneri:** Somut bir "basit mod" talebi olmadıkça **yapma** (bu dosyayı tasarım notu olarak tut).
Yapılırsa **ek seçenek** olarak eklenmeli, tam değişim değil.

### Adım Adım (yapılırsa)
1. **Mod seçici:** action/param `bool use_facade False`. `true` iken decomposition / TSP /
   custom-order istekleri **reddedilir** (net `INVALID_MODE_SET`) — facade bunları desteklemez.
2. **`f2c::Options` eşleme:** opennav mode mesajlarını Options alanlarına eşle
   (`hg_swaths`, `sg_obj`, `sg_angle`, `rp_alg`, `pp_alg`); `local_crs = cartesian_frame_`.
3. **Çağrı + dönüşüm:** `F2CPath p = f2c::planCovPath(robot, master_field, opt, cartesian_frame_);`
   → mevcut `toCoveragePathMsg`/`toNavPathMsg` (Path tipi aynı).

### B.8 Testler
- **T-B8-1:** facade modu basit convex alanda SUCCEEDED; çıktı granüler hatla benzer uzunlukta.
- **T-B8-2:** `use_facade=true` + decomposition/TSP → net hata (INVALID_MODE_SET).

### Riskler
- Facade decomposition/TSP-cap/headland-first yapamaz → "basit mod" olarak sınırla; gelişmiş
  goal'lar için mode reddi şart, yoksa sessizce eksik plan üretir.
