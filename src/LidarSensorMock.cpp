#include "../include/LidarSensorMock.h"

LidarSensorMock::LidarSensorMock(
    Distance zMin,
    Distance zMax,
    Distance d,
    unsigned int fovc
    ) : zMin(zMin), zMax(zMax), d(d), fovc(fovc) {};

LidarScanResult LidarSensorMock::getScan(const Orientation drone_orientation) {

}
