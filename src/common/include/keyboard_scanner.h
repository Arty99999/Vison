//
// Created by plutoli on 2022/1/26.
//

#ifndef CUBOT_BRAIN_KEYBOARD_SCANNER_H
#define CUBOT_BRAIN_KEYBOARD_SCANNER_H

#include <termio.h>
#include "easy_logger.h"

/**
 * @brief 键盘扫描器
 */
class KeyboardScanner
{
public:
    /**
     * @brief 不需要回车操作，直接扫描键盘按下的键值
     * @return 键盘按下的键值
     * @note 在CLion的调试状态下，“Run”窗口不是一般意义的终端，获取键盘按键的键值还需要使用回车键
     * @remark 参考网址：https://www.cnblogs.com/SchrodingerDoggy/p/14072739.html
     */
    static int GetPressedKeyWithoutEnter();
};

#endif //CUBOT_BRAIN_KEYBOARD_SCANNER_H