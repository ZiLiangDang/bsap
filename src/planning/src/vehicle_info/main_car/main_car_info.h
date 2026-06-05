#ifndef MAIN_CAR_BASE_H_
#define MAIN_CAR_BASE_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class MainCar : public VehicleBase // 主车
    {
    public:
        MainCar();
        // 定位点转frenet
        void vehicle_cartesian_to_frenet(const Referline &refer_line) override;
    };
} // namespace Planning
#endif // MAIN_CAR_BASE_H_