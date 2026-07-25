# F2C v2 — B.5: `HL_SWATH` Tipi Desteği

> **Durum:** Planlanıyor  
> **Ön koşul:** B.3 (Decomposition) tamamlanmış olmalı  
> **Bağımlılık:** B.3 → B.5 (HL_SWATH state'lerini yalnızca B.3 üretiyor; önce
> B.3 olmadan end-to-end test edilemez, uygulamak ölü kod eklemek olur)  
> **Efor:** 🟢 Küçük (~1-2 saat, B.3 sonrası)

---

## Nedir?

F2C v2'nin `PathSectionType` enum'u üç değer tanımlıyor:

```
SWATH    = 1  → iç şerit (kapsama geçişi)
TURN     = 2  → şeritler arası dönüş
HL_SWATH = 3  → headland (başlık bölgesi) şeridi
```

Şu anki `toCoveragePathMsg(Path)` implementasyonu (`utils.hpp` L182-186) yalnızca
`SWATH` ve `TURN`'ü tanıyor; başka herhangi bir türle karşılaşınca:

```
throw std::runtime_error("Unknown type of path state detected, cannot obtain path!")
```

B.3 (Decomposition) uygulandıktan sonra pipeline HL_SWATH state'leri üretmeye
başlar. Bu noktada mevcut throw coverage server'ı crash'e götürür.

**B.5'in görevi:** `toCoveragePathMsg()` state machine'ini HL_SWATH'ı tanıyacak
şekilde genişlet — HL_SWATH, SWATH gibi "sürülebilir şerit" semantiği taşıdığı
için aynı grupta işlenir.

---

## Neden B.3'ten Önce Değil?

- HL_SWATH state'leri **yalnızca B.3'ün** headland path generation'ı çalıştırmasıyla
  pipeline'a girer. B.3 olmadan hiç üretilmez.
- B.3 olmadan B.5'i yazmak: test edilemeyen ölü kod, end-to-end doğrulama imkânsız.
- B.3 ile birlikte tek PR'da sunmak mantıklı: B.3 üretir, B.5 tüketir, ikisi
  birlikte test edilir.

---

## Kapsam

**B.5 kapsamında:**
1. `toCoveragePathMsg(Path)`: HL_SWATH'ı SWATH gibi işle, throw yerine
2. `toNavPathMsg()`: kod değişikliği yok (zaten çalışıyor), sadece test
3. `TesttoCoveragePathMsg2`'deki eski throw assertion'ını sil

**B.5 KAPSAMI DIŞI** (B.3 sonrası ayrıca değerlendirilecek):
- Headland perimetrini otomatik path'e ekleme
- Coverage server'da "headland'i de sür" modu
- **HL_SWATH densification:** `discretizeSwath()` kaynak kodunda (`Path.cpp` L308)
  yalnızca `SWATH` tipini böler; `else` dalı tip kontrolü yapmadan her şeyi geçirir.
  HL_SWATH **garantili** olarak tek nokta olarak geçer. Pratik sonucu: 5 metrelik
  headland segmenti kontrolcüye tek waypoint görünür. Bu kabul edilmiş kısıtlama —
  densification ancak headland path'i pipeline'a gerçekten eklendiğinde anlamlı.

---

## Etkilenen Dosyalar

| Dosya | Değişiklik türü |
|---|---|
| `opennav_coverage/include/opennav_coverage/utils.hpp` | `toCoveragePathMsg(Path)` state machine |
| `opennav_coverage/test/test_utils.cpp` | Throw testi silme + yeni testler |

---

## Mevcut State Machine (`utils.hpp` L145–196)

`toCoveragePathMsg(Path)` şu an iki-durum FSM:

```
SWATH → SWATH   : devam (no-op)
TURN  → TURN    : poses'a ekle
SWATH → TURN    : swath kapat, turn başlat
TURN  → SWATH   : yeni swath start noktası kaydet
else            : throw  ← HL_SWATH buraya düşüyor, crash
```

---

## Adım Adım

### Adım 1 — `toCoveragePathMsg(Path)`: state machine genişletme

**Tasarım:** `isSwathLike()` yardımcı lambda ile SWATH ve HL_SWATH'ı aynı grupta
tut. Böylece mevcut mantık bozulmaz, yeni tip için tek tanım noktası olur.

```pseudo
isSwathLike(t) → t == SWATH || t == HL_SWATH
```

Genişletilmiş geçiş tablosu:

```
isSwathLike → isSwathLike : devam (no-op)
isSwathLike → TURN        : swath kapat, turn başlat
TURN        → isSwathLike : yeni swath start noktası kaydet
TURN        → TURN        : poses'a ekle
else                      : throw (gerçekten bilinmeyen tip için kalır)
```

Implementasyon pseudo kodu:

```pseudo
// İlk state:
if isSwathLike(path[0].type):
    curr_swath_start = path[0].point
elif path[0].type == TURN:
    yeni turn başlat

// Loop:
if isSwathLike(curr) && isSwathLike(new):
    no-op
elif isSwathLike(curr) && new == TURN:
    swath kapat, turn başlat
elif curr == TURN && isSwathLike(new):
    swath start kaydet
elif curr == TURN && new == TURN:
    pose ekle
else:
    throw

// Son:
if isSwathLike(curr):
    son swath kapat
```

### Adım 2 — `toNavPathMsg()`: kod değişikliği yok

`discretizeSwath()` zaten HL_SWATH'ı `else` dalından geçiriyor (tek nokta).
B.6 ile eklenen velocity/is_backward doldurma kodu `state.velocity` ve `state.dir`
alanlarını okuduğu için HL_SWATH'ı da otomatik kapsar. Kod değişikliği gerekmez.

### Adım 3 — `TesttoCoveragePathMsg2`: throw assertion'ını sil

Mevcut testte (L136-139):

```pseudo
path_in.getStates()[0].type = HL_SWATH;
EXPECT_THROW(toCoveragePathMsg(path_in, ...), std::runtime_error);
```

**Bu iki satırı sil.**

Gerekçe: `PathSectionType` bir `enum class`; F2C kendi enum'unu değiştirmeden
gerçek anlamda "bilinmeyen tip" ortaya çıkamaz. `static_cast<PathSectionType>(99)`
ile yapay geçersiz değer üretmek C++ standardı tarafından garanti edilmiyor
(implementation-defined) — kırılgan test, değer üretmiyor.

### Adım 4 — Yeni testler

**`TesttoCoveragePathMsgHLSwath` — parse doğrulaması:**

```pseudo
// Senaryo 1: sadece HL_SWATH
path: [HL_SWATH, HL_SWATH, HL_SWATH]
→ swaths.size() == 1, turns.size() == 0

// Senaryo 2: gerçekçi karışım (B.3 çıktısı buna benzer)
path: [HL_SWATH, HL_SWATH, TURN, TURN, SWATH, SWATH]
→ swaths.size() == 2, turns.size() == 1

// Senaryo 3: SWATH + HL_SWATH karışımı
path: [SWATH, SWATH, HL_SWATH, HL_SWATH, TURN, TURN, SWATH]
→ swaths.size() == 2, turns.size() == 1
  (SWATH+HL_SWATH tek sürekli blok sayılır — isSwathLike grubu)
```

**`TesttoNavPathMsgHLSwathVelocity` — velocity/is_backward kontrolü:**

```pseudo
// Bir HL_SWATH state (velocity=0.8, dir=FORWARD)
// discretizeSwath sonrası tek nokta olarak geçer
→ vels.size() == 1, vels[0] ≈ 0.8
→ dirs.size() == 1, dirs[0] == false
```

**Çalıştırma:**

```bash
colcon test --packages-select opennav_coverage \
            --ctest-args -R test_utils
```

---

## Dikkat Edilecekler

- **`HL_SWATH` v2.0.0'da var mı?** Evet —
  `Fields2Cover/include/fields2cover/types/PathState.h` L18: `HL_SWATH = 3`.
- **`discretizeSwath()` davranışı:** Kaynak kod doğrulandı (`Path.cpp` L305-322).
  `else` dalı tip kontrolsüz `addState` çağırır — HL_SWATH **garantili** tek nokta.
- **B.3 ile aynı PR mı?** Evet önerilir — B.3 üretir, B.5 tüketir, ikisi birlikte
  anlamlı. Ayrı commit'ler ama aynı PR.
- Commit mesajı: `"B.5: handle HL_SWATH in toCoveragePathMsg (with B.3)"`
