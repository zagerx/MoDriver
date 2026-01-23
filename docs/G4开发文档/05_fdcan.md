# STM32CUBEMX配置FDCAN外设

## 波特率设置

- 1M

## 过滤器
- 过滤器总数28个

## FIFO
- FIFO0(3条报文)
- FIFO1(3条报文)
- TxFIFO(9条报文)

### FIFO和过滤器联系
```C
FDCAN_FilterTypeDef sFilterConfig;

sFilterConfig.IdType = FDCAN_STANDARD_ID;          // 过滤标准ID
sFilterConfig.FilterIndex = 0;                     // 使用过滤器0
sFilterConfig.FilterType = FDCAN_FILTER_MASK;      // 使用掩码模式
sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 关键！路由到FIFO0
sFilterConfig.FilterID1 = 0x100;                   // 期望的ID：0x100
sFilterConfig.FilterID2 = 0x7F0;                   // 掩码：0x7F0

HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig); // 应用配置
```

## 中断设置
中断线
- FDCAN1_IT0_IRQn
  - 接收FIFO0 新报文
  - 发送完成
  - 发送FIFO空
  - 各种错误（仲裁、协议等）
- FDCAN1_IT1_IRQn
  - 接收FIFO1 新报文



