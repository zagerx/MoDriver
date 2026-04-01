/**
 * @file prot_voltage.c
 * @brief 此文件已被弃用，保留仅为兼容 Makefile
 * 
 * @note 电压保护已拆分为独立的模块：
 *       - prot_overvoltage.c  过压保护
 *       - prot_undervoltage.c 欠压保护
 * 
 * 原因：修复故障清除 bug（原设计中 PROT_TYPE_VOLTAGE 只有一个类型，
 * 但可能返回 FAULT_OVERVOLTAGE 或 FAULT_UNDERVOLTAGE，导致清除故障时
 * 只能清除其中一个位）
 */

// 此文件故意留空，仅为兼容 Makefile 中的引用
// 实际功能已移至 prot_overvoltage.c 和 prot_undervoltage.c
