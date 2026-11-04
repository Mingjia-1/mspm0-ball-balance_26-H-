/**
 * @file safety_manager.h
 * @brief 声明失线、视觉超时、限位、堵转和复位后的安全处理接口。
 */
#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

void SafetyManager_init(void);
void SafetyManager_process(void);

#endif
