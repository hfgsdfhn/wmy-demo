#include "app_navigation.h"

#include <stddef.h>
#include <string.h>

/* 当前导航目标点缓存 */
static PathPoint app_navigation_target;
/* 当前目标点是否有效 */
static bool app_navigation_target_valid;

/*
 * 初始化导航模块，清空目标点状态。
 */
bool AppNavigation_Init(void)
{
    memset(&app_navigation_target, 0, sizeof(app_navigation_target));
    app_navigation_target_valid = false;
    return true;
}

/*
 * 更新导航目标点。
 * 当新点有效且指针非空时，选择当前目标点并标记为有效。
 */
void AppNavigation_Step(const PathPoint *point,
                        bool point_available)
{
    if (point_available && (point != NULL))
    {
        app_navigation_target = *point;
        app_navigation_target_valid = true;
    }
}

/*
 * 读取当前导航目标点。
 * 如果当前目标点无效，或输出指针为空，则返回失败。
 */
bool AppNavigation_GetTarget(PathPoint *point)
{
    if (!app_navigation_target_valid || (point == NULL))
    {
        return false;
    }
    *point = app_navigation_target;
    return true;
}
