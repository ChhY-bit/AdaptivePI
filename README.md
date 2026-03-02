# 自适应PI控制器设计与仿真
---
## 项目简介

本项目介绍了针对典型一阶惯性系统的自适应PI控制器的设计方法，所提方法对具有未知或时变参数的一阶惯性系统有效。通过在线参数辨识技术，自动调整PI控制器参数，实现对被控对象的高性能控制。

## 适用场景

- 温度控制系统（如电炉、热水壶等）
- 液位控制系统
- 其他可近似为一阶惯性系统的工业控制对象

## 项目结构

```
AdaptivePID/
├── documents/                  # 技术文档目录
│   ├── First_Order_System.md       # 技术文档：通过详尽的理论阐明原理与方法
│   ├── First_Order_System.png      # 图片形式
│   ├── Second_Order_System.md      # 关于二阶系统的扩展，作为参考
│   └── Second_Order_System.png     # 图片形式
│
├── results/                    # 仿真结果数据
│   ├── 1_Results_IdealPI.mat       # 理想PI控制器仿真数据
│   ├── 2_Results_NormalPI.mat      # 常规PI控制器仿真数据（手动参数）
│   └── 3_Results_AdaptivePI.mat    # 自适应PI控制器仿真数据
│
├── main_simulation.m           # 主仿真程序（自适应PI控制器）
├── update_rk4.m                # 四阶龙格-库塔积分器
├── Preview.m                   # 绘图函数（预览仿真结果）
├── DrawResult.m                # 批量绘图程序（展示对比结果）
├── .gitignore                  # Git忽略文件配置
│
└── README.md                     # 本文件
```

---

**作者**：ChhY-bit
**日期**：2026-3-2
**联系方式**：1774808359@qq.com