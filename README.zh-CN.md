[English](README.md)

# APPlace

**APPlace** 是一个面向模拟电路的 routing-aware analytical placement
framework。与仅优化器件坐标并将 access-point (AP) 选择留给后续布线的
传统 analytical placement 不同，APPlace 在 global placement 中联合优化
**device locations 和 access points**，使布局阶段使用的互连几何更接近
router 最终实际采用的连接位置。

APPlace 包含两个主要阶段：

1. **Routing-Aware Global Placement**  
   将器件坐标和 AP 变量纳入同一个连续优化过程。APPlace 使用可微的
   terminal/AP 几何表示、AP-aware wirelength objective，以及自适应的
   multi-objective control，使器件位置与其最合适的访问位置共同演化。

2. **Structure-Preserving Legalization**  
   在消除 overlap、满足 symmetry 的同时，尽量保留 global placement
   已经形成的 routing-relevant structure。该阶段由
   reference-guided expansion 和 bounded local conflict resolution 组成。

论文中的实验在四个真实模拟电路 benchmark 上评估 APPlace。实验和
ablation 表明，AP co-optimization 与 structure-preserving legalization
都会显著影响最终 post-layout quality。

> **Paper:** *APPlace: Co-Optimizing Device Locations and Access Points
> for Routing-Aware Analog Placement*, accepted to ASP-DAC 2027,
> [doi:10.1145/3840404.3848293](https://doi.org/10.1145/3840404.3848293).

本仓库提供 APPlace 的 Rust 实现、无需 PDK 即可运行的 abstract benchmarks，
以及用于接入实际 PCell / PDK 数据的输入接口。本仓库只负责 placement；
routing、parasitic extraction 和 post-layout simulation 不包含在其中。

> 我们希望 APPlace 能为开源社区以及模拟电路设计自动化做出一点贡献。这个工具没有在我们的实验环境和实验所用的 PDK 之外做过大规模测试,遇到问题是正常的。欢迎在 issue 里提出,我们会在力所能及的范围内修复,也欢迎提交 PR。提 issue 或 PR 时,请附上能复现问题的具体输入,以便把它加入长期回归测试集。如果设计不便公开,提供一个几何形态的case(见 `docs/INPUT_FORMAT.md`)就够了,它不含任何工艺数据。

## 包含什么

- `src/` 下的三个 crate:`applace-core`(几何与网表模型)、
  `applace-placer`(梯度布局器与合法化器)、`applace-io`(解析器、
  写出器与命令行入口)。
- `benchmark/abstract/` 下的四个示例设计,除了 Rust 工具链之外
  不需要任何准备即可运行;每个 case 的 `reference/` 下附带本仓库对它产出的布局。
- `run/` 下的 shell 入口脚本,以及 `scripts/` 下两个跑完之后要用的工具:
  `plot_placement.py` 看结果,`check_reference.sh` 用参考不变量检查你的构建。

## 安装

APPlace 只需要一套标准的 Rust 工具链(edition 2021),此外别无依赖。
不依赖任何求解器,不依赖 C++,也没有 cargo feature 需要选:编译出来的就是全部。

```bash
# 如果还没有 Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 编译
cargo build --release --manifest-path src/Cargo.toml --workspace
```

编译产物默认写到 `./target`,可用 `TARGET_DIR` 改到别处。

## 快速开始

一条命令,不需要任何其他输入:

```bash
bash run/10-abstract-placement.sh ota2 /tmp/applace-ota2
```

这会对 `ota2` 示例做布局(49 个器件、35 条网络、24 组对称对),写出两个文件:

```
/tmp/applace-ota2/placement.json     器件 -> [x, y],外框左下角,单位纳米
/tmp/applace-ota2/placement.gds      同一份布局的外框级版图
```

用任意 GDS 查看器打开 `placement.gds` 就能看到结果:每个器件一个矩形、
一个写着器件名的文字标注、一个引脚锚点,以及布局区域。
`docs/OUTPUT.md` 给出图层分配与 JSON 的完整 schema。

另外三个示例是 `ota1`、`ota3`、`comp`,把名字换掉即可。

## 两种输入形态

APPlace 接受两种形态的设计输入。

| | 几何输入 | 器件输入 |
| --- | --- | --- |
| 你需要提供 | 四个小 JSON 文件:器件外框、引脚与网络表、对称对、参数 | 一份 SPICE 网表、每个器件一份 PCell GDS、一份规则 deck |
| 需要 PDK 吗 | 否 | 是 |
| 需要厂商 PCell 几何吗 | 否 | 是 |
| 入口 | `run/10-abstract-placement.sh` | `run/01-placement.sh` |
| 自带示例数据 | 有,四个设计 | 无,需自备 |

**几何输入**就是上面的快速开始。它把完整的布局问题化简为布局器真正需要的东西:
每个器件多大、什么连什么、哪些必须保持对称。自带的四个示例用的都是这种形态。

**器件输入**吃的是真实设计:带器件尺寸的层次化 SPICE 网表、一个存放逐器件
PCell 版图的目录,以及一份描述工艺的规则 deck。APPlace 解析 PCell 几何,
由此得到器件外框与引脚位置,然后运行同一个布局器。本仓库**不附带**
这种形态的设计。
`docs/INPUT_FORMAT.md` 逐项规定了器件流读取的每个文件,你可以据此组装
自己的 case 目录。

```bash
bash run/01-placement.sh /path/to/your/case /tmp/out
```

## 输出

几何流写出 `placement.json` 与 `placement.gds`;当最后的legalization环节发生
重试时,另写一份 `placement.lcr_retry.json`。

器件流对每个阶段都给出一份 JSON 坐标表和一份 GDS 版图:

| 阶段 | 含义 | 文件 |
| --- | --- | --- |
| Global Placement | 基于梯度的布局种子 | `global_placement.{json,gds}` |
| Reference-Guided Expansion | 第一个合法化阶段 | `reference_guided_expansion.{json,gds}` |
| Local Conflict Resolution | 最后的重叠修复 | `local_conflict_resolution.{json,gds}` |

`docs/OUTPUT.md` 对以上全部做了说明,包括 GDS 图层号与重试记录。

## 看懂这张图

`scripts/plot_placement.py` 把一次运行画成一张带标注的版图。对任意输出目录都能跑:

```bash
bash run/10-abstract-placement.sh comp out/comp
python3 scripts/plot_placement.py out/comp --out comp.png
```

布局结果本身只是一串坐标,光看坐标看不出这个结果合不合理。乍看「不合理」的版图,
通常由两件事解释,图里都画了出来。

**保护环预留。** 模拟电路里的 PMOS 器件要落在一圈 N 阱保护环内,而保护环是布局之后
才加的,所以布局器会在每个 PMOS 周围留出一圈空白。两个器件之间比外框需要的更远,
原因往往就在这里。图上用围着 PMOS 的虚线框画出这一圈,画的就是**实际预留的面积**,
不是画图时临时加的余量。

**对称。** 模拟电路是绕镜像轴摆的。每一对匹配器件用同一种填充色,穿过它们的那条轴
画的是**布局真正实现的轴**,不是声明的那条轴。哪一对没有关于它的轴对称,一眼就能看见。

| 标记 | 含义 |
| --- | --- |
| 橙色方框、粗边、写着 `PMOS` | 一个 PMOS 器件 |
| 灰色方框 | 其它器件 |
| 橙色虚线框 | 该 PMOS 周围留空的保护环预留区 |
| 彩色填充,一对一色 | 一组对称对 |
| 紫色点划线 | 实际实现的对称轴 |
| 蓝色虚线框 | 布局实际占用的外接矩形 |
| 绿色点线框 | 你声明的布局区域 |
| 红色轮廓 | 仍与别人重叠的器件 —— 不该出现 |
| 灰色星形连线 | 网络连接关系,从器件中心画起 |
| 深色小点 | 器件中心,几何形态在这里放置引脚初值 |

标题行给出该先看的几个数:器件数与 PMOS 数、外接矩形、利用率、**重叠对数(应为 0)**、
以及以器件中心计的 HPWL。同样这些数也写在 `placement_summary.json` 里,
不想看图可以直接读。

同一份版图也可以用任意 GDS 查看器打开 `placement.gds`;图层 4 与 5 承载预留区与
PMOS 标记(完整图层表见 `docs/OUTPUT.md`)。

## 参数

所有优化器设置都从 case 的 `params.json` 读取,并且都可以在命令行覆盖:

```bash
bash run/10-abstract-placement.sh ota2 /tmp/out --set symWeight=2.0
```

`docs/PARAMETERS.md` 列出了每一个键的类型、默认值与作用。

## 已知限制

在判断 APPlace 是否适用于你的问题之前,请先读这一节。

- **只做布局。** 不布线,不做布局后几何(保护环、电源条、衬底接触),
  不做寄生提取,不做仿真。
- **不包含任何工艺数据。** `pdk/example_lef.json` 里每一个规则数值都是编造的占位值。
  唯一的例外是 `pmos`/`nmos`/`cap`/`res` 四个列表:它们写的是器件 model 名,
  必须与你自己网表里的 model 字符串对得上,器件分类才会生效。
  要跑真实工艺,请自行提供真实PDK信息和Pcell GDS;`docs/INPUT_FORMAT.md` 说明了详细内容。
- **器件流不附带示例设计。** 只有几何形态的示例随仓库发布。
- **几何形态不施加工艺间距。** 没有规则 deck 就没有器件间最小间距,几何形态的
  布局里器件外框可能贴合。只有器件形态在给定 deck 时才会从工艺规则推出合法化的
  间距下限;DRC 意义来自这条路径。
- **对称约束。** 目前对称轴仅支持一个对称组共享一条竖轴或横轴。
  旋转对称,以及超出「共享一条轴」的层次对称,都不在本仓库已实现模型之内。
- **非层次化布局。** 网表中的层次在布局前被拍平到器件级;
  APPlace 暂时不把子块当作块来摆放。
- **两个参数是预留的,目前不起任何作用。** `useWellCluster`、`enableArray`
  会被接受并校验,但它们对应的惩罚项尚未实现:贡献恒为零,
  四个示例 case 在这两个开关的任意取值下都产出逐字节相同的布局。
  为了让已有参数文件继续可加载,它们在 `docs/PARAMETERS.md` 中标注为预留,
  而不是被删掉。

## 引用

如果 APPlace 对你的研究有帮助，请引用：

```bibtex
@inproceedings{liu2027applace,
  author    = {Liu, Bingyang and Gao, Xiaohan and Poddar, Souradip and Pan, David Z.},
  title     = {{APPlace}: Co-Optimizing Device Locations and Access Points for Routing-Aware Analog Placement},
  booktitle = {Proceedings of the Asia and South Pacific Design Automation Conference (ASP-DAC)},
  year      = {2027},
  doi       = {10.1145/3840404.3848293},
  note      = {To appear},
}
```

## 致谢

本工作部分得到 NSF(项目号 CCF-2112665)、Samsung、UT Austin iMAGiNE consortium
以及 NVIDIA 设备捐赠的支持。另外感谢北京大学的林亦波和张昊懿的帮助。

## 许可

Copyright (c) 2026 The University of Texas at Austin
All rights reserved.

采用 GNU General Public License v3.0(GPL-3.0)许可,见 [LICENSE](LICENSE)。

除非另行声明,提交以供合入的任何贡献都按上述同一许可发布,
不附加额外条款。
