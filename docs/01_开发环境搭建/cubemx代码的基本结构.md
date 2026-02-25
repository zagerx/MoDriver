# 基础结构
- `main.c`
- `gpio.c`
- `tim.c`
- `hal_xx_tim.c` HAL库文件

## `main.c`的定位
- `mian.c`应该位于APP顶层，完全和底层无关
- 应该对其进行改造，不再将其放到`stmcubemx.a`这个库里面

## `stm32cubemx.a`作为最底层
- 提供HAL库

## 总结

