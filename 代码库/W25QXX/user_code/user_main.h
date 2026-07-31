#ifndef USER_MAIN_H
#define USER_MAIN_H

/* 主程序初始化：在硬件外设初始化完成后调用一次。 */
void init(void);

/* 主循环任务：在 while(1) 中持续调用。 */
void loop(void);

#endif /* USER_MAIN_H */
