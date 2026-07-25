# F2C v2 — B.3 Decomposition + B.5 HL_SWATH (Birlikte PR)

> **Durum:** Planlanıyor
> **Branch:** `Feature/umutc/f2c-enhancements-main` (mevcut branch) veya yeni branch
> **Ön koşul:** B.4 ✅, B.6 ✅, B.7 ✅ tamamlandı
> **Bağımlılık:** B.3 → B.5 (B.3 HL_SWATH state'leri üretir, B.5 tüketir — aynı PR)
> **Bağımsız:** B.1/B.2 gerekmez — B.3 standalone yaklaşımla flatten+mevcut route pipeline kullanır
> **Efor:** 🟠 Orta (~3-4 gün: B.3 büyük kısmı, B.5 küçük)

---

## Bağlam — Neden B.3 + B.5 Birlikte?

B.3 (decomposition) non-convex tarlayı convex alt hücrelere böler. Bu süreç F2C
pipeline'ında `HL_SWATH` tipinde `PathState`'ler üretir. Mevcut `toCoveragePathMsg()`
(`utils.hpp` L182-186) bu tipi görünce `throw` ediyor → coverage server crash.

B.5, bu crash'i `HL_SWATH`'ı `SWATH` gibi işleyerek düzeltir. Dolayısıyla B.3 olmadan
B.5 ölü kod, B.5 olmadan B.3 çalışır coverage server olmaz.

---

## B.3 Mimari Kararı — Standalone (B.1 Bağımsız)

**Yaklaşım:**
```
F2CCell field_no_headland
    → F2CCells (wrapper: addGeometry)
    → f2c::decomp::TrapezoidalDecomp / BoustrophedonDecomp → F2CCells (n hücre)
    → SwathGenerator::generateSwaths(F2CCells) → F2CSwathsByCells
    → .flatten() → F2CSwaths
    → mevcut RouteGenerator pipeline (değişmez)
```

F2C zaten `generateBestSwaths(obj, width, F2CCells) → F2CSwathsByCells` sağlıyor.
`SwathsByCells::flatten() → F2CSwaths` mevcut; tek-hücre pipeline'a geçiş trivial.

**Trade-off:** Hücreler arası geçiş sırası TSP optimal değil (B.1 ile düzelir, sonraya
bırakılıyor). Tek-poligon kapsama için zaten mevcut `BoustrophedonOrder` ile yeterli.

---

## Etkilenen Dosyalar

### Yeni Dosyalar

| Dosya | İçerik |
|---|---|
| `opennav_coverage_msgs/msg/DecompMode.msg` | `type` + `split_angle` alanları |
| `opennav_coverage/include/opennav_coverage/decomp_generator.hpp` | `DecompGenerator` sınıfı |
| `opennav_coverage/src/decomp_generator.cpp` | `decompose()` implementasyonu |
| `opennav_coverage/test/test_decomp_generator.cpp` | decomp birim testleri |

### Değiştirilen Dosyalar

| Dosya | Değişiklik |
|---|---|
| `opennav_coverage/include/opennav_coverage/types.hpp` | `DecompType` enum ekle |
| `opennav_coverage_msgs/action/ComputeCoveragePath.action` | `generate_decomp` bool + `decomp_mode` alanı |
| `opennav_coverage_msgs/CMakeLists.txt` | `DecompMode.msg` kayıt |
| `opennav_coverage/include/opennav_coverage/headland_generator.hpp` | `generateHeadlands(F2CCells)` overload |
| `opennav_coverage/src/headland_generator.cpp` | multi-cell headland implementasyonu |
| `opennav_coverage/include/opennav_coverage/swath_generator.hpp` | `generateSwaths(F2CCells)` overload |
| `opennav_coverage/src/swath_generator.cpp` | multi-cell implementasyonu |
| `opennav_coverage/include/opennav_coverage/coverage_server.hpp` | `decomp_gen_` member + `default_generate_decomp_` |
| `opennav_coverage/src/coverage_server.cpp` | `default_generate_decomp` parametresi + decomp akışı |
| `opennav_coverage_bt/include/opennav_coverage_bt/compute_complete_coverage_path.hpp` | `generate_decomp`, `decomp_mode_type`, `decomp_split_angle` portları |
| `opennav_coverage_bt/src/compute_complete_coverage_path.cpp` | `on_tick()` port okuma |
| `opennav_coverage_bt/test/test_compute_coverage_path.cpp` | decomp port testi |
| `opennav_coverage_demo/params/demo_params.yaml` | `default_generate_decomp`, `default_decomp_type`, `default_decomp_split_angle` |
| `opennav_coverage/CMakeLists.txt` | `decomp_generator.cpp` kaynak + test |
| `opennav_coverage/include/opennav_coverage/utils.hpp` | **B.5** — `toCoveragePathMsg` state machine |
| `opennav_coverage/test/test_utils.cpp` | **B.5** — throw assertion sil + yeni testler |

---

## Adım Adım Uygulama

### Adım 1 — `types.hpp`: `DecompType` enum

`PathContinuityType` enum'unun hemen altına ekle:

```cpp
/**
 * @enum Decomposition types
 */
enum class DecompType
{
  UNKNOWN = 0,
  NONE = 1,
  TRAPEZOIDAL = 2,
  BOUSTROPHEDON = 3
};
```

---

### Adım 2 — `DecompMode.msg`

`opennav_coverage_msgs/msg/DecompMode.msg` oluştur.

> ⚠️ Mevcut tüm msg'lar (`HeadlandMode`, `SwathMode`, `RouteMode`, `PathMode`) `string mode "UNKNOWN"`
> desenini kullanıyor, `uint8 type` değil. `DecompMode` da aynı deseni izlemeli — aksi hâlde
> `settings.type == DecompMode::UNKNOWN` gibi bir sabit olmaz ve derleme hatası çıkar.

```
string mode "UNKNOWN"  # NONE, TRAPEZOIDAL, BOUSTROPHEDON
float64 split_angle 0.0
```

`opennav_coverage_msgs/CMakeLists.txt`'e kayıt:
```cmake
"msg/DecompMode.msg"
```
(diğer `.msg` kayıtlarının yanına alfabetik sırada)

---

### Adım 3 — `ComputeCoveragePath.action`: yeni goal alanları

`bool generate_headland True` satırından önce:

```
bool generate_decomp False
opennav_coverage_msgs/DecompMode decomp_mode
```

---

### Adım 4 — `decomp_generator.hpp`

`headland_generator.hpp` desenini kopyala, adaptasyonlar:

```cpp
#ifndef OPENNAV_COVERAGE__DECOMP_GENERATOR_HPP_
#define OPENNAV_COVERAGE__DECOMP_GENERATOR_HPP_

#include "fields2cover.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "opennav_coverage_msgs/msg/decomp_mode.hpp"
#include "opennav_coverage/types.hpp"
#include "opennav_coverage/utils.hpp"  // util::toUpper (decomp_generator.cpp'de kullanılır)

namespace opennav_coverage
{

class DecompGenerator
{
public:
  template<typename NodeT>
  explicit DecompGenerator(const NodeT & node)
  {
    logger_ = node->get_logger();
    nav2::declare_parameter_if_not_declared(
      node, "default_decomp_type", rclcpp::ParameterValue("NONE"));
    std::string type_str = node->get_parameter("default_decomp_type").as_string();
    default_type_ = toType(type_str);

    nav2::declare_parameter_if_not_declared(
      node, "default_decomp_split_angle", rclcpp::ParameterValue(0.0));
    default_split_angle_ = node->get_parameter("default_decomp_split_angle").as_double();
  }

  // Ana metot: F2CCells → F2CCells (alt hücrelere bölünmüş)
  F2CCells decompose(
    const F2CCells & cells,
    const opennav_coverage_msgs::msg::DecompMode & settings);

  void setMode(const std::string & new_mode);
  void setSplitAngle(double angle) { default_split_angle_ = angle; }

protected:
  std::string toString(const DecompType & type);
  DecompType toType(const std::string & str);

  DecompType default_type_;
  double default_split_angle_;
  rclcpp::Logger logger_{rclcpp::get_logger("DecompGenerator")};
};

}  // namespace opennav_coverage

#endif
```

---

### Adım 5 — `decomp_generator.cpp`

```cpp
#include "opennav_coverage/decomp_generator.hpp"

namespace opennav_coverage
{

F2CCells DecompGenerator::decompose(
  const F2CCells & cells,
  const opennav_coverage_msgs::msg::DecompMode & settings)
{
  // "UNKNOWN" → default_type_ kullan (HeadlandGenerator/SwathGenerator deseniyle aynı)
  DecompType type = toType(settings.mode);
  if (type == DecompType::UNKNOWN) {
    type = default_type_;
  }

  double split_angle =
    std::abs(settings.split_angle) < 1e-9 ?
    default_split_angle_ : settings.split_angle;

  if (type == DecompType::NONE) {
    return cells;  // no-op
  }

  RCLCPP_DEBUG(
    logger_, "Decomposing field with type %s, split_angle=%.3f",
    toString(type).c_str(), split_angle);

  if (type == DecompType::TRAPEZOIDAL) {
    f2c::decomp::TrapezoidalDecomp decomp;
    decomp.setSplitAngle(split_angle);
    return decomp.decompose(cells);
  } else if (type == DecompType::BOUSTROPHEDON) {
    f2c::decomp::BoustrophedonDecomp decomp;
    decomp.setSplitAngle(split_angle);
    return decomp.decompose(cells);
  }

  throw CoverageException("Unknown decomp type requested!");
}

std::string DecompGenerator::toString(const DecompType & type)
{
  switch (type) {
    case DecompType::NONE: return "NONE";
    case DecompType::TRAPEZOIDAL: return "TRAPEZOIDAL";
    case DecompType::BOUSTROPHEDON: return "BOUSTROPHEDON";
    default: return "UNKNOWN";
  }
}

DecompType DecompGenerator::toType(const std::string & str)
{
  std::string upper = str;
  util::toUpper(upper);
  if (upper == "NONE") return DecompType::NONE;
  if (upper == "TRAPEZOIDAL") return DecompType::TRAPEZOIDAL;
  if (upper == "BOUSTROPHEDON") return DecompType::BOUSTROPHEDON;
  return DecompType::UNKNOWN;  // caller'a bırakılır (default'a düşer)
}

void DecompGenerator::setMode(const std::string & new_mode)
{
  default_type_ = toType(new_mode);
}

}  // namespace opennav_coverage
```

---

### Adım 6 — `swath_generator.hpp/cpp`: multi-cell overload (gerçek switch yapısıyla)

Mevcut `generateSwaths(Field, mode)` içindeki type/angle çözümleme mantığı
([swath_generator.cpp:26-43](opennav_coverage/src/swath_generator.cpp#L26)) tek-hücre ve çok-hücre
arasında **birebir aynı** — yalnızca son F2C çağrısı (`field` vs `cells` + `.flatten()`) farklı.
Bu ortak mantığı bir private helper'a (`resolveSwathParams`) çıkar; iki public overload yalnızca
F2C çağrısında ayrışsın. Mevcut davranış **birebir korunur**.

**Header** (`swath_generator.hpp`) — public overload + protected helper:

```cpp
// public: mevcut generateSwaths(Field, ...) imzasının hemen altına
Swaths generateSwaths(
  const F2CCells & cells, const opennav_coverage_msgs::msg::SwathMode & settings);

// protected: çözümlenmiş parametreleri taşıyan yardımcı yapı + metot
struct ResolvedSwathParams
{
  SwathObjectivePtr objective;
  SwathAngleType angle_type;
  float swath_angle;
  float step_angle;
};
ResolvedSwathParams resolveSwathParams(
  const opennav_coverage_msgs::msg::SwathMode & settings);
```

**`swath_generator.cpp`** — mevcut `generateSwaths(Field, ...)` gövdesini helper + ince overload'a böl:

```cpp
SwathGenerator::ResolvedSwathParams SwathGenerator::resolveSwathParams(
  const opennav_coverage_msgs::msg::SwathMode & settings)
{
  SwathType action_type = toType(settings.objective);
  SwathAngleType action_angle_type = toAngleType(settings.mode);
  ResolvedSwathParams p;

  // If not set by action, use default mode (mevcut mantık, aynen)
  if (action_type == SwathType::UNKNOWN && action_angle_type == SwathAngleType::UNKNOWN) {
    action_type = default_type_;
    p.angle_type = default_angle_type_;
    p.objective = default_objective_;
    p.swath_angle = default_swath_angle_;
    p.step_angle = default_step_angle_;
  } else {
    p.angle_type = action_angle_type;
    p.objective = createObjective(action_type);
    p.swath_angle = settings.best_angle;
    p.step_angle = settings.step_angle;
  }

  RCLCPP_DEBUG(
    logger_, "Generating Swaths with: %s", toString(action_type, p.angle_type).c_str());
  return p;
}

Swaths SwathGenerator::generateSwaths(
  const Field & field, const opennav_coverage_msgs::msg::SwathMode & settings)
{
  ResolvedSwathParams p = resolveSwathParams(settings);
  const double op_width = robot_params_->getOperationWidth();
  generator_->setAllowOverlap(default_allow_overlap_);
  switch (p.angle_type) {
    case SwathAngleType::BRUTE_FORCE:
      if (!p.objective) {
        throw CoverageException("No valid swath mode set! Options: LENGTH, NUMBER, COVERAGE.");
      }
      generator_->setStepAngle(p.step_angle);
      return generator_->generateBestSwaths(*p.objective, op_width, field);
    case SwathAngleType::SET_ANGLE:
      return generator_->generateSwaths(p.swath_angle, op_width, field);
    default:
      throw CoverageException("No valid swath angle mode set! Options: BRUTE_FORCE, SET_ANGLE.");
  }
}

Swaths SwathGenerator::generateSwaths(
  const F2CCells & cells, const opennav_coverage_msgs::msg::SwathMode & settings)
{
  // Tek hücre → mevcut Field yolunu kullan (flatten gereksiz)
  if (cells.size() == 1) {
    return generateSwaths(cells.getGeometry(0), settings);
  }

  ResolvedSwathParams p = resolveSwathParams(settings);
  const double op_width = robot_params_->getOperationWidth();
  generator_->setAllowOverlap(default_allow_overlap_);
  switch (p.angle_type) {
    case SwathAngleType::BRUTE_FORCE:
      if (!p.objective) {
        throw CoverageException("No valid swath mode set! Options: LENGTH, NUMBER, COVERAGE.");
      }
      generator_->setStepAngle(p.step_angle);
      return generator_->generateBestSwaths(*p.objective, op_width, cells).flatten();
    case SwathAngleType::SET_ANGLE:
      return generator_->generateSwaths(p.swath_angle, op_width, cells).flatten();
    default:
      throw CoverageException("No valid swath angle mode set! Options: BRUTE_FORCE, SET_ANGLE.");
  }
}
```

> F2C base `generateBestSwaths/generateSwaths(..., const F2CCells&)` → `F2CSwathsByCells` döndürür;
> `.flatten()` → `F2CSwaths` (= `Swaths`) verir ([swath_generator_base.h:25-32](Fields2Cover/include/fields2cover/swath_generator/swath_generator_base.h#L25)). İki küçük switch'in
> kopyalanması bilinçli — tek fark F2C çağrısının argümanı; template helper ile tekilleştirmek
> okunabilirliği bozar, over-abstraction'a gerek yok.

---

### Adım 7 — `headland_generator.hpp/cpp`: multi-cell overload

Mevcut `generateHeadlands(Field, mode) → Field` metodunun yanına yeni overload:

**Header** (`headland_generator.hpp`) — mevcut imzanın hemen altına:

```cpp
// Multi-cell overload: her sub-cell için ayrı headland uygula
F2CCells generateHeadlands(
  const F2CCells & cells,
  const opennav_coverage_msgs::msg::HeadlandMode & settings);
```

**`headland_generator.cpp`** implementasyonu — mevcut tek-hücre metodunu içeriden çağırır,
F2C'ye hiç dokunmaz:

```cpp
F2CCells HeadlandGenerator::generateHeadlands(
  const F2CCells & cells,
  const opennav_coverage_msgs::msg::HeadlandMode & settings)
{
  F2CCells result;
  for (size_t i = 0; i < cells.size(); ++i) {
    result.addGeometry(generateHeadlands(cells.getGeometry(i), settings));
  }
  return result;
}
```

---

### Adım 8 — `coverage_server.hpp/cpp`: decomp akışı + YAML parametresi

**`coverage_server.hpp`** — yeni member + generator:

```cpp
std::unique_ptr<DecompGenerator> decomp_gen_;
bool default_generate_decomp_{false};
```

**`on_configure`** — diğer generator'larla birlikte:

```cpp
decomp_gen_ = std::make_unique<DecompGenerator>(node);

nav2::declare_parameter_if_not_declared(
  node, "default_generate_decomp", rclcpp::ParameterValue(false));
get_parameter("default_generate_decomp", default_generate_decomp_);
```

**`computeCoveragePath`** — mevcut (1)/(2) bloklarının
([coverage_server.cpp:189-196](opennav_coverage/src/coverage_server.cpp#L189)) yerine.

**Tek kapı kararı (YAML kontrolü):** decomp yalnızca `do_decomp = goal->generate_decomp ||
default_generate_decomp_` ile açılır. Bu tek koşul planın **her yerinde** geçerlidir; ikinci bir
`if (goal->generate_decomp)` dalı hiçbir yerde yazılmaz. Böylece BT XML'e dokunmadan yalnızca
`default_generate_decomp: true` (Adım 10) ile tüm görevlerde decomp açılabilir.

```cpp
// (1) Optional: decompose non-convex field, then remove headland, then generate swaths
const bool do_decomp = goal->generate_decomp || default_generate_decomp_;

Field field_no_headland = field;   // non-decomp yolu + görselleştirme için korunur
Swaths swaths;
if (do_decomp) {
  F2CCells raw_cells;
  raw_cells.addGeometry(field);
  F2CCells decomposed = decomp_gen_->decompose(raw_cells, goal->decomp_mode);

  // Her sub-cell'e ayrı headland (bkz. Dikkat Edilecekler — decomp→headland sırası)
  F2CCells cells_no_headland = decomposed;
  if (goal->generate_headland) {
    cells_no_headland = headland_gen_->generateHeadlands(decomposed, goal->headland_mode);
  }
  swaths = swath_gen_->generateSwaths(cells_no_headland, goal->swath_mode);
  // Not: field_no_headland = field kalır → görselleştirme toplam sınır + swath'ları gösterir.
  // Decomposed hücrelerin ayrı görselleştirmesi bu PR kapsamı dışı (bkz.
  // F2C_V2_GUNCELLEME_VE_YENI_OZELLIKLER.md → B.3, deferred görselleştirme).
} else {
  if (goal->generate_headland) {
    field_no_headland = headland_gen_->generateHeadlands(field, goal->headland_mode);
  }
  swaths = swath_gen_->generateSwaths(field_no_headland, goal->swath_mode);
}
```

Sonraki (3)/(4) route/path blokları ve satır 234'teki `visualizer_->visualize(field,
field_no_headland, ...)` çağrısı **olduğu gibi kalır** — decomp yolunda `field_no_headland == field`
olduğundan görselleştirme sorunsuz çalışır, visualizer'a hiç dokunulmaz.

---

### Adım 9 — BT node güncellemesi

**Hedef:** BT XML üzerinden `generate_decomp`, `decomp_mode.mode`, `decomp_split_angle`'ı
per-mission override edebilelim.

**`compute_complete_coverage_path.hpp`** — `providedPorts()`'a ekle:

```cpp
BT::InputPort<bool>("generate_decomp", false, "Whether to decompose non-convex field"),
BT::InputPort<std::string>("decomp_mode_type", "UNKNOWN", "NONE/TRAPEZOIDAL/BOUSTROPHEDON"),
BT::InputPort<double>("decomp_split_angle", 0.0, "Split angle in radians"),
```

**`compute_complete_coverage_path.cpp`** — `on_tick()`'e ekle:

```cpp
getInput("generate_decomp", goal_.generate_decomp);
std::string decomp_type;
getInput("decomp_mode_type", decomp_type);
goal_.decomp_mode.mode = decomp_type;
getInput("decomp_split_angle", goal_.decomp_mode.split_angle);
```

**BT XML** (`navigate_w_basic_complete_coverage.xml`) — port defaults yeterli,
XML'e ek yazma zorunlu değil. Override için:

```xml
<ComputeCoveragePath
  generate_decomp="true"
  decomp_mode_type="TRAPEZOIDAL"
  decomp_split_angle="1.5708"
  nav_path="{path}" .../>
```

---

### Adım 10 — `demo_params.yaml` güncellemesi

`coverage_server.ros__parameters` altına ekle:

```yaml
coverage_server:
  ros__parameters:
    # ... mevcut parametreler ...
    default_generate_decomp: false       # true yapınca tüm görevlerde decomp açılır
    default_decomp_type: "NONE"          # NONE / TRAPEZOIDAL / BOUSTROPHEDON
    default_decomp_split_angle: 0.0      # radyan; 0 = swath açısına paralel bölme
```

> Bu üç parametre birlikte YAML'dan tam kontrol sağlar — BT XML'e dokunmak gerekmez.

---

### ~~Adım 11 — visualizer~~ (bu PR'dan çıkarıldı)

Decomposed hücrelerin ayrı `MarkerArray` ile görselleştirilmesi bu PR kapsamından çıkarıldı;
gelecek bir geliştirme olarak **[F2C_V2_GUNCELLEME_VE_YENI_OZELLIKLER.md](F2C_V2_GUNCELLEME_VE_YENI_OZELLIKLER.md) → B.3**
altına (frame_id düzeltilmiş haliyle) taşındı. Bu PR'da visualizer'a **hiç dokunulmaz**; decomp
yolunda mevcut `visualize(field, field_no_headland, ...)` çağrısı `field_no_headland == field` ile
toplam sınır + swath'ları gösterir (Adım 8). Adım numaraları geriye dönük tutarlılık için korunmuştur.

---

### Adım 12 — `CMakeLists.txt`: yeni kaynak + test


`opennav_coverage/CMakeLists.txt`'e:

```cmake
# kaynak listesine ekle (mevcut .cpp'lerin yanına)
  src/decomp_generator.cpp   # <-- ekle
```

Test:
```cmake
ament_add_gtest(test_decomp_generator test/test_decomp_generator.cpp)
target_link_libraries(test_decomp_generator coverage_server ...)
```

---

### Adım 13 — `test_decomp_generator.cpp`: birim testler

`test_headland.cpp`'deki `HeadlandShim` desenini birebir izle.

**DecompShim** (dosyanın başına, namespace içine):

```cpp
class DecompShim : public DecompGenerator
{
public:
  template<typename NodeT>
  explicit DecompShim(const NodeT & node) : DecompGenerator(node) {}

  std::string toStringShim(const DecompType & type)
  { return toString(type); }

  DecompType toTypeShim(const std::string & str)
  { return toType(str); }
};
```

**TestDecompUtils** — toString/toType/setterlar:

```cpp
TEST(DecompTests, TestDecompUtils)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto gen = DecompShim(node);

  // toType: bilinmeyen → UNKNOWN (throw değil)
  EXPECT_EQ(gen.toTypeShim("FAKE"), DecompType::UNKNOWN);
  EXPECT_EQ(gen.toTypeShim("UNKNOWN"), DecompType::UNKNOWN);

  // toType: büyük/küçük harf
  EXPECT_EQ(gen.toTypeShim("NONE"), DecompType::NONE);
  EXPECT_EQ(gen.toTypeShim("none"), DecompType::NONE);
  EXPECT_EQ(gen.toTypeShim("TRAPEZOIDAL"), DecompType::TRAPEZOIDAL);
  EXPECT_EQ(gen.toTypeShim("trapezoidal"), DecompType::TRAPEZOIDAL);
  EXPECT_EQ(gen.toTypeShim("BOUSTROPHEDON"), DecompType::BOUSTROPHEDON);
  EXPECT_EQ(gen.toTypeShim("boustrophedon"), DecompType::BOUSTROPHEDON);

  // toString
  EXPECT_EQ(gen.toStringShim(DecompType::NONE), std::string("NONE"));
  EXPECT_EQ(gen.toStringShim(DecompType::TRAPEZOIDAL), std::string("TRAPEZOIDAL"));
  EXPECT_EQ(gen.toStringShim(DecompType::BOUSTROPHEDON), std::string("BOUSTROPHEDON"));
  EXPECT_GT(gen.toStringShim(DecompType::UNKNOWN).size(), 0u);  // "UNKNOWN" veya warn msg

  // setter'lar throw etmemeli
  gen.setMode("TRAPEZOIDAL");
  gen.setMode("none");
  gen.setSplitAngle(0.5 * M_PI);
}
```

**TestDecompNone** — NONE modu alanı değiştirmemeli:

```cpp
TEST(DecompTests, TestDecompNone)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto gen = DecompShim(node);
  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells; cells.addGeometry(field.getField().getGeometry(0));

  DecompMode mode; mode.mode = "NONE";
  F2CCells result = gen.decompose(cells, mode);
  EXPECT_EQ(result.size(), 1u);
}
```

**TestDecompNonConvex** — non-convex L-şekilli tarla gerçekten bölünmeli:

```cpp
TEST(DecompTests, TestDecompNonConvex)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto gen = DecompShim(node);

  // Elle L-şekilli non-convex poligon yap (F2C tutorial'dan)
  F2CLinearRing ring;
  ring.addPoint(0, 0); ring.addPoint(10, 0); ring.addPoint(10, 5);
  ring.addPoint(5, 5);  ring.addPoint(5, 10); ring.addPoint(0, 10);
  ring.addPoint(0, 0);  // kapalı halka
  F2CCell cell; cell.addRing(ring);
  F2CCells cells; cells.addGeometry(cell);

  DecompMode mode;
  mode.mode = "TRAPEZOIDAL";
  F2CCells result_trap = gen.decompose(cells, mode);
  EXPECT_GT(result_trap.size(), 1u);  // L-tarla bölünmeli

  mode.mode = "BOUSTROPHEDON";
  F2CCells result_bou = gen.decompose(cells, mode);
  EXPECT_GT(result_bou.size(), 1u);
}
```

**TestDecompDefaultFallback** — `mode="UNKNOWN"` → default_type_ (NONE) kullanır:

```cpp
TEST(DecompTests, TestDecompDefaultFallback)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto gen = DecompShim(node);
  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  F2CCells cells; cells.addGeometry(field.getField().getGeometry(0));

  DecompMode mode;  // default mode = "UNKNOWN" → default_type_ = NONE
  F2CCells result = gen.decompose(cells, mode);
  EXPECT_EQ(result.size(), cells.size());  // NONE gibi davranır, değişmez
}
```

---

### Adım 14 — B.5: `utils.hpp` state machine genişletme

**Mevcut** `toCoveragePathMsg(Path)` içindeki `throw` dalı:

```cpp
} else {
  throw std::runtime_error("Unknown type of path state detected, cannot obtain path!");
}
```

**Değişiklik:** `isSwathLike` lambda ekle, tüm `PathSectionType::SWATH` karşılaştırmalarını
bunu kullanacak şekilde güncelle:

```cpp
auto isSwathLike = [](f2c::types::PathSectionType t) {
  return t == f2c::types::PathSectionType::SWATH ||
         t == f2c::types::PathSectionType::HL_SWATH;
};
```

Geçiş tablosu (önceki → sonraki):

```
isSwathLike → isSwathLike : devam (no-op)
isSwathLike → TURN        : swath kapat, turn başlat
TURN        → isSwathLike : yeni swath start noktası kaydet
TURN        → TURN        : pose ekle
else                      : throw (gerçekten bilinmeyen tip için kalır)
```

---

### Adım 15 — B.5: `test_utils.cpp` güncellemesi

**Silme** (`TesttoCoveragePathMsg2` içinde, HL_SWATH throw satırları):
```cpp
path_in.getStates()[0].type = HL_SWATH;
EXPECT_THROW(toCoveragePathMsg(path_in, ...), std::runtime_error);
```

**Yeni test** — `TesttoCoveragePathMsgHLSwath` (4 senaryo, hepsi `swaths_ordered`/`contains_turns` flag'lerini de kontrol etmeli):

```cpp
// Yardımcı: path state üret
auto makeState = [](f2c::types::PathSectionType t) {
  PathState s; s.type = t; s.point = F2CPoint(0, 0); s.angle = 0.0; return s;
};
using PST = f2c::types::PathSectionType;

// Senaryo 1: sadece HL_SWATH
// path: [HL_SWATH, HL_SWATH, HL_SWATH]
// → swaths.size()==1, turns.size()==0, swaths_ordered==true, contains_turns==true (bak: mevcut impl)
//   (veya false — mevcut davranışı gözlemle; flag'leri EXPECT_EQ ile sabitle)

// Senaryo 2: HL_SWATH → TURN → SWATH (B.3 gerçekçi çıktısı)
// path: [HL_SWATH, HL_SWATH, TURN, TURN, SWATH, SWATH]
// → swaths.size()==2, turns.size()==1, turns[0].poses.size()==2

// Senaryo 3: SWATH + HL_SWATH karışımı (isSwathLike grubu)
// path: [SWATH, SWATH, HL_SWATH, HL_SWATH, TURN, TURN, SWATH]
// → swaths.size()==2, turns.size()==1
// (SWATH→HL_SWATH geçişi no-op: aynı swath bloğu devam eder)

// Senaryo 4: TURN ile başlayıp HL_SWATH'a geçiş (TURN → isSwathLike kenarı)
// path: [TURN, TURN, HL_SWATH, HL_SWATH]
// → turns.size()==1, swaths.size()==1
// ⚠️ Bu senaryo state machine'deki "TURN → isSwathLike" geçişini doğrudan test eder
```

> `swaths_ordered` ve `contains_turns` flag değerleri için mevcut `TesttoCoveragePathMsg2`
> testinin L131-133'teki EXPECT_EQ satırlarına bak; aynı flag kontrollerini yeni senaryolara uygula.

**velocity testi (B.6'ya taşındı):** `TesttoNavPathMsgHLSwathVelocity` bu commit'e **koyma** —
`toNavPathMsg()` kodu değişmiyor, test B.6 commit'ine veya mevcut `TesttoNavPathMsg`'e satır
olarak eklenmeli.

---

### Adım 16 — `test_headland.cpp`: F2CCells overload testi

`TestheadlandGeneration` testinin altına yeni test ekle:

```cpp
TEST(HeadlandTests, TestheadlandGenerationMultiCell)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto generator = HeadlandShim(node);

  f2c::Random rand;
  auto field = rand.generateRandField(1e5, 5);
  Field cell = field.getField().getGeometry(0);
  double area_in = cell.area();

  // 2 hücrelik F2CCells
  F2CCells cells;
  cells.addGeometry(cell);
  cells.addGeometry(cell);

  opennav_coverage_msgs::msg::HeadlandMode settings;
  F2CCells result = generator.generateHeadlands(cells, settings);

  EXPECT_EQ(result.size(), 2u);
  // Her hücre küçülmüş olmalı (headland alındı)
  EXPECT_LT(result.getGeometry(0).area(), area_in);
  EXPECT_LT(result.getGeometry(1).area(), area_in);
}
```

---

### Adım 17 — `test_compute_coverage_path.cpp`: BT node decomp port testi

Mevcut `test_tick` testinin altına yeni test ekle:

```cpp
TEST_F(ComputeCoveragePathActionTestFixture, test_decomp_ports)
{
  // generate_decomp=true ve decomp_mode_type BT XML'den geçmeli
  std::string xml_txt =
    R"(
      <root BTCPP_format="4" main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <ComputeCoveragePath
              nav_path="{path}"
              generate_decomp="true"
              decomp_mode_type="TRAPEZOIDAL"
              decomp_split_angle="1.5708"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // Mock server'a ulaşan goal'da generate_decomp=true olduğu doğrulanabilirse
  // action_server_->getReceivedGoal().generate_decomp == true kontrol et
  // (TestActionServer altyapısı goal'u saklıyorsa — yoksa SUCCESS yeterli)

  tree_->haltTree();
}
```

---

### Adım 18 — `test_server.cpp`: decomp yolu server testi

Mevcut `TestServerTransactions` testine ek olarak:

```cpp
TEST(CoverageTests, TestDecompPath)
{
  // Mevcut test gibi: server node başlat, action client oluştur
  // ...

  auto goal_msg = ComputeCoveragePath::Goal();
  goal_msg.generate_decomp = true;
  // mode alanını doldur
  goal_msg.decomp_mode.mode = "TRAPEZOIDAL";
  goal_msg.generate_headland = true;
  goal_msg.generate_route = true;
  goal_msg.generate_path = false;
  // Mevcut testteki GML dosyası veya polygon girdisini kullan
  // (test_server.cpp'deki mevcut goal kurulum kodunu aynen kopyala)

  // ...
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

// Senaryo 2: generate_headland=false (cells_no_headland = decomposed dalını test eder)
TEST(CoverageTests, TestDecompPathNoHeadland)
{
  goal_msg.generate_decomp = true;
  goal_msg.decomp_mode.mode = "BOUSTROPHEDON";
  goal_msg.generate_headland = false;  // ← cells_no_headland = decomposed dalını tetikler
  goal_msg.generate_route = false;
  goal_msg.generate_path = false;
  // ...
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

// Senaryo 3: YAML parametresi — default_generate_decomp=true ile goal.generate_decomp=false
// Server parametresini set ederek OR mantığını doğrula
// (test_server.cpp'de node parametresini set etme mekanizmasına bak)
// goal_msg.generate_decomp = false (default), ama server parametresi true → decomp çalışmalı
// EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
```

---

## Build & Test Sırası

```bash
# 1. Msgs paketi önce (yeni DecompMode.msg)
colcon build --packages-select opennav_coverage_msgs

# 2. Coverage paketi
colcon build --packages-select opennav_coverage

# 3. Hedefli testler (yeni dosyalar + değiştirilen dosyalar)
colcon test --packages-select opennav_coverage \
            --ctest-args -R "test_decomp_generator|test_utils|test_headland|test_server"

# 4. BT paketi
colcon build --packages-select opennav_coverage_bt
colcon test --packages-select opennav_coverage_bt \
            --ctest-args -R "test_compute_coverage_path"

# 5. Geniş kapsam
colcon build --packages-select \
  opennav_coverage_msgs opennav_coverage opennav_row_coverage \
  opennav_coverage_bt opennav_coverage_demo opennav_coverage_navigator
colcon test --packages-select opennav_coverage opennav_row_coverage opennav_coverage_bt
```

---

## Commit Yapısı

> **Akış:** Tek PR açılır; iş commit-commit yapılır ve hepsi birlikte review edilir. Aşağıdaki
> sıra bağımlılık yönünde ilerler — her commit **kendi başına derlenir ve kendi testini geçirir**.
> B.5 (HL_SWATH), server'ı gerçekten HL_SWATH üreten yola bağlayan commit'ten **önce** gelir; böylece
> hiçbir commit'te server crash edebilir bir ara durum oluşmaz.

**PR: `F2C v2 B.3 (decomposition) + B.5 (HL_SWATH handling)`**

1. `B.3: add DecompType enum and DecompMode msg`
   - Adım 1 (types.hpp enum), 2 (DecompMode.msg + msgs CMakeLists)
2. `B.3: add generate_decomp fields to ComputeCoveragePath action`
   - Adım 3 (action alanları)
3. `B.3: add DecompGenerator`
   - Adım 4 (hpp), 5 (cpp), 12 (CMakeLists kaynak+test), 13 (test_decomp_generator)
4. `B.3: add multi-cell swath generator overload`
   - Adım 6 (resolveSwathParams refactor + F2CCells overload) + ilgili swath test eklentisi
5. `B.3: add multi-cell headland generator overload`
   - Adım 7 (headland hpp/cpp), 16 (test_headland)
6. `B.5: handle HL_SWATH in toCoveragePathMsg`
   - Adım 14 (utils.hpp state machine), 15 (test_utils.cpp)
7. `B.3: wire decomposition into coverage server with YAML control`
   - Adım 8 (coverage_server hpp/cpp tek-kapı akışı), 10 (demo_params), 18 (test_server)
8. `B.3: add BT node ports for decomposition control`
   - Adım 9 (BT node hpp/cpp), 17 (test_compute_coverage_path)

> Visualizer commit'i (eski #3) kaldırıldı → [F2C_V2_GUNCELLEME_VE_YENI_OZELLIKLER.md](F2C_V2_GUNCELLEME_VE_YENI_OZELLIKLER.md) B.3'e taşındı.
> Daha da bölmek istersen commit 3'ü "DecompGenerator core" + "DecompGenerator tests" olarak,
> commit 7'yi "server wiring" + "server test" olarak ikiye ayırabilirsin.

---

## Dikkat Edilecekler

- **`f2c::decomp::DecompAlg` enum'unda TRAPEZOIDAL/BOUSTROPHEDON yorum satırı:**
  `fields2cover.h` L131-132'de yorum — bu F2C'nin facade `Options` için; granüler
  API (`TrapezoidalDecomp` / `BoustrophedonDecomp` sınıfları) kullanılabilir.
- **Decompose → headland sırası (semantik karar — bilinçli seçim):**
  Akış ham `field`'ı önce decompose eder, **sonra her sub-cell'e ayrı** headland uygular
  (F2C tutorial sırası). Bunun somut sonucu: headland offset'i yalnızca tarlanın dış sınırına
  değil, **decomposition'ın ürettiği iç kesim çizgilerine de** uygulanır → robot, hücreler
  arasındaki sanal kesiklerde de headland payı bırakır. Bu, F2C'nin önerdiği ve çok-hücreli
  rota (B.1) ile uyumlu olan davranıştır; hücreler bağımsız kapsanacağı için mantıklıdır.
  **Alternatif** (önce tüm tarlaya headland, sonra decompose) iç kesiklerde pay bırakmaz ama
  B.1 çok-hücreli rota ile daha zor eşleşir. Bu PR tutorial sırasını seçer; alternatif, B.1
  entegrasyonunda yeniden değerlendirilebilir. **Not:** tek convex tarlada (decomp'un tek hücre
  ürettiği durum) iki sıra da aynı sonucu verir, fark yalnızca gerçekten bölünen tarlalarda görülür.
- **`SwathsByCells::flatten()` sırası:** Hücreler arası geçiş sıralaması TSP optimal
  değil (B.1 olmadan). Kabul edilmiş kısıtlama.
- **`DecompObjective` kullanılmıyor (v1 kapsamı):** F2C `decompose(cells, obj)`'un ikinci
  parametresi opsiyonel `obj::DecompObjective`; bu PR varsayılanı (default-construct) kullanır.
  Objektif tabanlı decomposition ayarı bilinçli olarak kapsam dışıdır — gerekirse `DecompMode.msg`'a
  ileride alan eklenerek açılabilir.
- **B.5 `HL_SWATH` densification kısıtlaması:** `discretizeSwath()` yalnızca `SWATH`
  tipini böler; HL_SWATH tek nokta olarak geçer. Kabul edilmiş kısıtlama.
- **Row coverage etkilenmez:** `opennav_row_coverage` F2C decomp kullanmıyor.
- **`validateGoal` — değişiklik gerekmiyor (netleştirildi):** `generate_decomp=true` +
  `generate_headland=false` **geçerli** bir kombinasyondur (Adım 8 `else if` dalı bunu zaten
  ele alır: decomposed hücreler doğrudan swath'lanır). Bu yüzden `validateGoal`'a yeni kısıt
  eklenmez. `decomp_mode.mode` boş/bilinmeyen gelirse `DecompGenerator` default'a düşer (throw yok).
- **split_angle fonksiyonel testi yok (bilinçli atlandı):** Farklı angle'ların
  farklı decomp sonucu verdiğini doğrulamak F2C iç davranışını test etmek olur —
  proje felsefesiyle çelişir. `setSplitAngle()` setter throw etmiyor testlendi, yeterli.
- **Senaryo 1 flag değerleri:** `toCoveragePathMsg(Path)` implementasyonunda
  `contains_turns` ve `swaths_ordered` flag'leri yalnızca TURN state'i görüldüğünde
  set ediliyor (mevcut koda göre). Sadece HL_SWATH içeren path için: `contains_turns=true`
  (init değeri, bak: mevcut impl), `swaths_ordered=true`. Uygulama sırasında mevcut
  kodun gerçek çıktısını gözlemleyip EXPECT_EQ ile sabitle — "gözlemle" soyut kalmasın.
- **F2CLinearRing/addPoint/addRing API:** Adım 12'deki L-şekil testinde kullanılan
  çağrıların F2C v2.0.0 gerçek metodlarıyla örtüşüp örtüşmediğini build sırasında
  teyit et. F2C test dosyalarında (`Fields2Cover/tests/cpp/`) örneklere bak.

---

## Sonraki Adım (Bu PR Sonrası)

B.1 (çok-hücreli TSP rota): `SwathsByCells.flatten()` yerine gerçek
`RoutePlannerBase::genRoute(cells, swaths_by_cells)` kullanımı. Bu PR'da
`decomposed` (`F2CCells`) ve `F2CSwathsByCells` üretimi ayrı tutulursa B.1 geçişi
daha temiz olur — ilerisi için not.
