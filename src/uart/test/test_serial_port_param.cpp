#include "serial_port_param.h"

int main(int, char* [])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 从yaml文件中加载通信串口参数
    SerialPortParam serialPortParam;
    SerialPortParam::LoadFromYamlFile("config/sentry/2021-09-05/param/serial_port_param.yaml",
                                      &serialPortParam);

    return 0;
}