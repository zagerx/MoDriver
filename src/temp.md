
# 立创转kicad
## 安装easyeda2kicad
- [说明文档](https://www.schemalyzer.com/zh/blog/easyeda/export-import/easyeda-to-kicad)

## 转换所有内容（符号 + 封装 + 3D 模型）
`easyeda2kicad --full --lcsc_id=C2040`
## 仅转换符号和封装
`easyeda2kicad --symbol --footprint --lcsc_id=C2040`
## 仅转换符号
`easyeda2kicad --symbol --lcsc_id=C2040`
## 仅转换封装
`easyeda2kicad --footprint --lcsc_id=C2040`
## 仅转换 3D 模型
`easyeda2kicad --3d --lcsc_id=C2040`
## 自定义输出路径
`easyeda2kicad --full --lcsc_id=C2040 --output ~/libs/my_lib`