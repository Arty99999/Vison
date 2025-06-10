//
// Created by cubot on 2022/6/16.
//

#include "predictor_command.h"
#include"robot_body_request.h"

int main()
{
    std::vector<PredictorCommand> BodyPredictorCommand;
    unsigned char frame[21];

    frame[0] = 0xAA;
    frame[1] = 0x07;

    frame[2] = 0x00;

    frame[3] = 0x01;
    frame[4] = 0x00;

    frame[5] = 0x01;
    frame[6] = 0x00;
    frame[7] = 0x00;
    frame[8] = 0x00;

    frame[9] = 0x01;

    frame[10] =  0x01;
    frame[11] = 0xA0;

    frame[12] = 0x01;
    frame[13] = 0xA0;

    frame[14] = 0x01;
    frame[15] = 0xA0;

    frame[16] = 0x01;
    frame[17] = 0xA0;

    frame[18] = 0x01;
    frame[19] = 0x08;

    frame[20] = 0xDD;

    std::chrono::time_point<std::chrono::steady_clock> beginTime = std::chrono::steady_clock::now();
    uint64_t beginTimestamp = beginTime.time_since_epoch().count();

    ERequestType type;
    if (SystemConfigurator::ConvertToRequestType(frame[1], &type))
    {
        // 解析机器人本体请求
        bool result = true;
        RobotBodyRequest request;
        request.Type = type;
        for (unsigned int i = 2; i < 20; ++i)
        {
            request.Datas.emplace_back(frame[i]);
        }

        // 处理机器人本体请求
        if (request.Type == ERequestType::PredictorCommond)
        {
            if (request.Datas.size() != 18)
            {
                result = false;
                std::cout<<"size of data is wrong"<<std::endl;
            }
            if(result)
            {
                PredictorCommand predictCommand;
                if (!PredictorCommand::Parse(request.Datas, &predictCommand))
                {
                    std::cout << "PredictorCommond parsed was failed." << std::endl;
                }
                else
                {
                    // 如果内存数据过多则更新
                    if (BodyPredictorCommand.size() > 1000)
                    {
                        // 则刷前半个容器
                        BodyPredictorCommand.erase(BodyPredictorCommand.begin(),BodyPredictorCommand.begin()+500);

                        // 更新数据
                        BodyPredictorCommand.push_back(predictCommand);

                        std::cout<<"erase half of vector"<<std::endl;
                    }
                    else
                    {
                        // 更新数据
                        BodyPredictorCommand.emplace_back(predictCommand);
                        std::cout<<"There is new data"<<std::endl;

                    }
                }
            }
        }
    }

    // 打印处理时间
    std::chrono::time_point<std::chrono::steady_clock> endTime = std::chrono::steady_clock::now();
    uint64_t endTimestamp = endTime.time_since_epoch().count();

    uint64_t processTime = endTimestamp - beginTimestamp;

    std::cout<<"processTime: "<<static_cast<double>(processTime)/1000000<<"ms"<<std::endl;
}
