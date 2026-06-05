#ifndef OBS_CAR_BASE_H_
#define OBS_CAR_BASE_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class ObsCar : public VehicleBase // 障碍车
    {
    public:
        ObsCar(const int &id);
        // 定位点转frenet
        void vehicle_cartesian_to_frenet(const Referline &refer_line) override;
    };
} // namespace Planning
#endif // OBS_CAR_BASE_H_