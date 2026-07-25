# F2C v2 — B.4: `NSwathModified` Objektifi

> **Durum:** Planlanıyor  
> **Ön koşul:** B.6 + B.7 tamamlandı (commit `06752a7`, `eb2374b`)  
> **Bağımlılık:** Yok — zincirden bağımsız, doğrudan uygulanabilir  
> **Efor:** 🟢 Küçük (~1-2 saat)

---

## Nedir?

`NSwathModified`, F2C v2'nin `f2c::obj` namespace'inde tanımladığı yeni bir swath
sayısı objektifidir. `NSwath`'ı miras alır ve `isFastCompAvailable() = true` döner;
bu, kapalı-form formülüyle brute-force döngüsü olmadan maliyet hesapladığı anlamına
gelir.

Pratik etkisi: `BRUTE_FORCE` açı modunda **belirgin biçimde daha hızlı** ve
az-dönüş optimizasyonu için tavsiye edilen varsayılan. F2C'nin kendi önerilen
varsayılanı bu objektif üzerine kurulu — bkz. `FIELDS2COVER_OZELLIK_KOD_HARITASI.md`
satır 561.

Şu anki `NUMBER` (`NSwath`) ile farkı davranışsal değil, **performans** farklıdır:
- `NSwath::isFastCompAvailable() = false` → her açı adımında tam hesap yapar
- `NSwathModified::isFastCompAvailable() = true` → analitik tahmin kullanır

---

## Etkilenen Dosyalar

| Dosya | Değişiklik türü |
|---|---|
| `opennav_coverage/include/opennav_coverage/types.hpp` | Enum değeri ekleme |
| `opennav_coverage/src/swath_generator.cpp` | 3 fonksiyon güncelleme |
| `opennav_coverage_msgs/msg/SwathMode.msg` | Yorum satırı güncelleme |
| `opennav_coverage/test/test_swath.cpp` | Test assertion'ları ekleme |

---

## Adım Adım

### Adım 1 — `types.hpp`: `SwathType` enum'una yeni değer ekle

`SwathType` enum'u şu an:

```
UNKNOWN  = 0
LENGTH   = 1
NUMBER   = 2
COVERAGE = 3
```

`NUMBER_MODIFIED = 4` değerini sonuna ekle. Mevcut sayısal değerler korunmalı.

### Adım 2 — `swath_generator.cpp`: `createObjective()`

Mevcut switch'e yeni case ekle:

```
case SwathType::LENGTH          → SwathLength
case SwathType::NUMBER          → NSwath
case SwathType::COVERAGE        → FieldCoverage
case SwathType::NUMBER_MODIFIED → NSwathModified   ← EKLE
default                         → nullptr (uyarı logla)
```

`f2c::obj::NSwathModified`, `fields2cover.h` umbrella header'ı üzerinden zaten
erişilebilir — ek `#include` gerekmez.

### Adım 3 — `swath_generator.cpp`: `toType()`

String → enum dönüşümü. `util::toUpper()` zaten uygulanıyor; `"NUMBER_MODIFIED"`
string'ini `SwathType::NUMBER_MODIFIED`'a eşle. Küçük harf varyantı otomatik çalışır.

### Adım 4 — `swath_generator.cpp`: `toString()`

Loglama için `SwathType::NUMBER_MODIFIED` → `"Number Modified"` string'i ekle.

> **Dikkat:** `toString()` çıktısı `"<tip> Objective and <açı> angle."` formatında.
> `"Number Modified"` (15 karakter), `"Number"` (6 karakter) yerine geçiyor —
> 9 karakterlik fark, mevcut string-boyut assert'lerini etkiler. Test yazarken
> yeni uzunluğu hesapla.

### Adım 5 — `SwathMode.msg`: yorum satırı güncelle

```
string objective "UNKNOWN"  # LENGTH, NUMBER, NUMBER_MODIFIED, or COVERAGE
```

### Adım 6 — Parametre dökümantasyonu (opsiyonel)

`coverage_server.hpp`'daki `default_swath_type` parametresi açıklaması
`NUMBER_MODIFIED`'ı içermiyorsa güncelle.

---

## Test Stratejisi

**Mevcut altyapı (`test_swath.cpp`):**

`SwathShim` sınıfı `protected` metodları `public` expose ediyor:
`createObjectiveShim`, `toTypeShim`, `toAngleTypeShim`, `toStringShim`.
`TestswathUtils` bu shim'lerle kapsamlı kontrol yapıyor — yeni assertion'lar
bu teste eklenir, yeni test fonksiyonu açılmaz.

**Eklenecek assertion'lar — `TestswathUtils`:**

```pseudo
// toType round-trip
EXPECT_EQ(toTypeShim("NUMBER_MODIFIED"), SwathType::NUMBER_MODIFIED)
EXPECT_EQ(toTypeShim("number_modified"), SwathType::NUMBER_MODIFIED)  // küçük harf

// createObjective — nullptr olmamalı
EXPECT_TRUE(createObjectiveShim(SwathType::NUMBER_MODIFIED))

// toString — uzunluk kontrolü (çalıştırarak hesapla, sabit yaz)
EXPECT_EQ(toStringShim(SwathType::NUMBER_MODIFIED, SwathAngleType::BRUTE_FORCE).size(), X)
```

**`TestswathGeneration`'a eklenen senaryo:**

```pseudo
settings.mode = "BRUTE_FORCE"
settings.objective = "NUMBER_MODIFIED"
auto swaths = generator.generateSwaths(field, settings)
// throw olmaması yeterli — sayısal doğruluk F2C'ye ait
```

**Özel sayısal test gerekli mi?** Hayır. F2C'nin kendi test paketi
`NSwathModified`'ın matematiksel doğruluğunu kapsamlı test ediyor; wrapper
sadece doğru nesneyi instantiate etmekten sorumlu.

**Çalıştırma:**

```bash
colcon test --packages-select opennav_coverage \
            --ctest-args -R test_swath
```

---

## Dikkat Edilecekler

- `NSwathModified::computeCost` override'ı `F2CCell` alıyor; `f2c::sg::BruteForce`
  bu imzayı biliyor — API uyumu var, ekstra adaptasyon gerekmez.
- `isFastCompAvailable() = true` olduğunda BruteForce generator analitik kod
  yoluna girer. `step_angle` ayarı daha az kritik hale gelir; yine de UI'dan
  kabul etmeye devam et.
- Commit mesajı: `"B.4: add NSwathModified swath objective"`
