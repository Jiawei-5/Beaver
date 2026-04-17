# Beaver v1.4 使用说明

## 1. 简介

Beaver 是一个面向 Abaqus/CAE 的 RVE 自动化插件，主要流程为：

- 批量读取 STL
- 自动平移 / 必要时按上限比例缩放颗粒
- 与立方体基体布尔合并并网格化
- 调用 EasyPBC 进行 6 个工况计算
- 自动复制并整理 ODB
- 基于 ODB 提取体积平均应力 / 应变并计算等效刚度矩阵

图形界面支持以下 4 个按钮：

- **Build Only**：仅建模
- **EasyPBC Only**：仅运行 EasyPBC
- **Stiffness Only**：仅进行结果提取与刚度矩阵输出
- **Run All**：完整一键流程

---

## 2. 推荐目录结构

### 方案 A：本机已经安装了 `stlImport` 和 `3DMesh_to_Geometry`（推荐）

如果当前电脑的 Abaqus 插件目录中已经存在以下文件夹：

- `stlImport`
- `3DMesh_to_Geometry`

则推荐使用下面这种更干净的目录结构：

```text
abaqus_plugins/
    Beaver/
        Beaver.py
        Beaver_plugin.py
        Beaver.ico
        README_Beaver_v1.4.md
    stlImport/
        ...
    3DMesh_to_Geometry/
        ...
```

在这种情况下：

- `Beaver/_deps` **不是必须的**
- 如果不需要做“便携分发”，**可以直接删掉 `_deps`**
- 这样通常可以减少或避免 Abaqus 启动时出现大量 duplicate warnings

### 方案 B：需要打包给没有安装相关插件的电脑使用

如果目标电脑没有安装 `stlImport` 和 `3DMesh_to_Geometry`，则可以保留 `_deps` 目录，把相关 `pyc` 一起打包：

```text
abaqus_plugins/
    Beaver/
        Beaver.py
        Beaver_plugin.py
        Beaver.ico
        _deps/
            stl_Constants.pyc
            stl_Import_plugin.pyc
            stl_ImportDB.pyc
            stl_ImportForm.pyc
            stl2inp.pyc
            dcel.pyc
            f3d_xgem.pyc
            mesh_geo.pyc
            mesh_geo_plugin.pyc
            mesh_geoDB.pyc
            sat_writer.pyc
```

### `_deps` 是否可以删除？

**可以，但前提是当前电脑已经能从外部找到这些依赖模块。**

建议这样理解：

- **本机自用，且已安装官方插件**：`_deps` 可以删
- **要发给其他电脑使用，且不确定对方有没有装插件**：保留 `_deps`

---

## 3. 插件放在哪里更稳

推荐优先放到**用户插件目录**，而不是直接放到 Abaqus 安装目录。常见示例：

```text
C:\Users\<用户名>\abaqus_plugins\Beaver
```

这样做的优点：

- 更容易被 Abaqus 自动加入 `sys.path`
- `import Beaver` 更容易成功
- 从 **Plug-ins** 菜单启动更稳定
- 不容易被安装目录权限影响

如果你已经确认放在安装目录里也能正常使用，也可以继续沿用。

---

## 4. 启动方式

### 方式 1：从 Plug-ins 菜单启动（推荐）

启动 Abaqus/CAE 后，从菜单进入 Beaver。

如果菜单里能看到 Beaver 但点击没有反应，常见原因通常不是 `Beaver.py` 本身，而是：

- 当前会话的 `sys.path` 中没有 Beaver 所在目录
- 插件目录层级不对
- `Beaver.py` 文件名没有保持为 `Beaver.py`
- Abaqus 没有重新启动，仍在使用旧的插件注册状态

### 方式 2：Run Script 启动

在 Abaqus/CAE 中直接运行 `Beaver.py`。

这种方式通常只要文件本身路径正确即可，适合排查插件注册问题。

### 方式 3：Kernel 手动导入测试

如果要排查路径问题，可在 Abaqus kernel 中手动执行：

```python
import sys
sys.path.insert(0, r'你的Beaver目录')
import Beaver
Beaver.launch_gui()
```

如果这样能成功，而 Plug-ins 菜单打不开，通常说明：

- Beaver 主脚本是正常的
- 问题在插件入口或模块搜索路径，而不是主逻辑

---

## 5. 图标说明

Beaver v1.4 支持自定义 Tk 窗口图标。

### 推荐格式

Windows 下最稳的是：

- `Beaver.ico`

也支持：

- `Beaver.png`
- `Beaver.jpg`
- `Beaver.jpeg`
- `Beaver.gif`

### 推荐做法

把图标文件放在 `Beaver.py` 同目录，并优先使用 `.ico`。

例如：

```text
Beaver/
    Beaver.py
    Beaver_plugin.py
    Beaver.ico
```

如果你已经有自己的图案，可以直接替换同名图标文件。

---

## 6. 界面参数说明

当前 v1.4 界面主要分为 4 组参数：

### I/O

- **Work directory (ODB/CSV)**：工作目录，输出日志、ODB、CSV、刚度矩阵等文件
- **CAE file name**：保存的 `.cae` 文件名
- **STL input directory**：输入 STL 文件夹

### Geometry / Mesh

- **Cube Lx / Ly / Lz**：RVE 立方体尺寸
- **Mesh size**：网格尺寸
- **Max shrink**：颗粒允许的最大均匀缩小比例，例如 `0.05 = 5%`

### Material (Elastic)

- **Matrix E / nu**：基体弹性模量、泊松比
- **Fibre E / nu**：颗粒 / 纤维弹性模量、泊松比

### EasyPBC

- **CPU count**：EasyPBC 求解用 CPU 数
- **Mesh sensitivity**：EasyPBC 配对节点时使用的几何容差

### 已移除的输入项

当前版本已经不再要求手动输入以下内容：

- `DELTA_E11 / DELTA_E22 / ... / DELTA_G12`
- 密度 `rho`
- `stlImport` 路径
- `3DMesh_to_Geometry` 路径

---

## 7. 运行后会输出什么

### 1）日志

默认输出到：

```text
<WORK_DIR>/logs/Beaver_YYYYmmdd_HHMMSS_PID.log
```

### 2）临时运行目录

EasyPBC 每个模型会在以下目录下使用独立临时目录运行：

```text
<WORK_DIR>/_beaver_run/
```

这样做的目的是避免上一次计算残留的 `.lck` 文件直接卡住下一次循环。

### 3）主要结果文件

对于每个模型，通常会得到：

```text
<Model>_E11.odb
<Model>_E22.odb
<Model>_E33.odb
<Model>_G23.odb
<Model>_G13.odb
<Model>_G12.odb
C_<Model>.txt
C_<Model>.npy
```

### 4）汇总文件

工作目录下通常还会生成：

```text
C_all_models.csv
fit_report.csv
```

其中：

- `C_all_models.csv`：全部模型的刚度矩阵汇总
- `fit_report.csv`：每个 STL 在装入立方体过程中的平移 / 缩放 / 跳过记录

---

## 8. 关于 duplicate warnings

如果你同时保留了：

- 系统安装的 `stlImport` / `3DMesh_to_Geometry`
- Beaver 自带 `_deps` 中的同名 `pyc`

Abaqus 启动时可能会出现类似下面的警告：

- `Duplicate plug-in file name ...`
- `Duplicate file name ...`

这类警告的含义通常是：

> 发现了多个同名模块或插件入口文件，Abaqus 只会优先使用它先找到的那一份。

### 是否有影响？

- **通常不一定会立刻导致失败**
- 但会让模块来源变得不够确定
- 也会让启动日志很乱

### 如何减少这类警告？

如果本机已经安装好了相关插件，最简单的方法就是：

- **删除 Beaver/_deps 中重复的依赖文件**

---

## 9. 常见问题

### Q1：Run Script 能跑，但 Plug-ins 点了没反应

常见原因：

- Beaver 目录没有被加入 `sys.path`
- 插件放置目录不合适
- `Beaver.py` 名称不正确
- 修改后没有重启 Abaqus/CAE

建议优先确认：

```python
import Beaver
```

是否能在 Abaqus kernel 中直接成功。

---

### Q2：`import Beaver.py` 为什么报错？

因为 Python 导入模块时**不能带 `.py` 后缀**。

正确写法是：

```python
import Beaver
```

错误写法是：

```python
import Beaver.py
```

---

### Q3：`_deps` 删除后如果又报缺依赖怎么办？

说明当前环境下 Beaver 还没有成功从外部插件目录找到以下模块：

- `stl2inp`
- `mesh_geo`
- 以及它们对应依赖

此时有两种处理方式：

1. **恢复 `_deps`**
2. 确认 `stlImport` 与 `3DMesh_to_Geometry` 是否确实安装，并且路径能被 Abaqus 识别

---

### Q4：出现 `abaqus.rpy` 权限警告怎么办？

例如：

```text
Permission was denied for abaqus.rpy; abaqus.rpy.xxx will be used
```

这通常只是 Abaqus 的 replay 文件无法写入默认名字，改用了另一个名字。

- 一般**不影响 Beaver 本体运行**
- 更像是工作目录权限或文件占用问题

---

## 10. 推荐使用策略

### 个人电脑 / 固定工作站

推荐：

- Beaver 单独放一个文件夹
- 不复制重复的依赖
- 直接使用系统已安装的 `stlImport` 和 `3DMesh_to_Geometry`
- `_deps` 可以删除

### 对外分发 / 交给别的电脑使用

推荐：

- 保留 `_deps`
- 把依赖一起打包
- 降低“对方电脑没装插件”的风险

---

## 11. 版本备注

本 README 适用于 **Beaver v1.4** 当前整合版使用方式，包含以下核心思路：

- 英文化后的主脚本与界面
- 以插件方式使用
- 支持自定义图标
- 使用 `_beaver_run` 独立临时目录降低 `.lck` 残留干扰
- 结果后处理直接基于 ODB 计算等效刚度矩阵
- `_deps` 仅作为可选便携依赖目录，不再强制要求长期保留

---

## 12. 建议随包保留的文件

如果你要给以后自己或别人长期使用，建议插件文件夹至少保留：

```text
Beaver.py
Beaver_plugin.py
Beaver.ico
README_Beaver_v1.4.md
```

如果需要便携分发，再加上：

```text
_deps/
```

---

如果后续 Beaver 继续改版，建议同步更新本 README，尤其是以下几项：

- 图标文件名
- `_deps` 搜索逻辑
- Plug-ins 启动入口
- 输出文件命名规则
- 结果后处理设置
