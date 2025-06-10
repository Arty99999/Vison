#include "huaray_camera_param.h"

int main(int argc, char *argv[])
{
    // 初始化日志记录器
    EasyLogger &logger = EasyLogger::GetSingleInstance();
    logger.Init();

    // 从yaml文件中加载华睿相机参数
    HuarayCameraParam param;
    HuarayCameraParam::LoadFromYamlFile("config/infantry_3/basement/huaray_camera_param.yaml",
                                        &param);

    return 0;
}