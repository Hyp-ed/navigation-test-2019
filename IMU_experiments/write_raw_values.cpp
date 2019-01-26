#include "sensors/imu.hpp"
#include "utils/timer.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"

#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>

using hyped::sensors::Imu;
using hyped::utils::Logger;
using hyped::utils::Timer;
using hyped::utils::ScopedTimer;
using hyped::utils::concurrent::Thread;
using hyped::data::ImuData;
using hyped::data::NavigationType;
using hyped::data::NavigationVector;

void sensorAverage(NavigationVector &acc, NavigationVector &gyr, std::vector<ImuData *> & msmnts)
{
  float nSensors = float(msmnts.size());
  for (ImuData *msmnt : msmnts)
  {
    acc[0] += msmnt->acc[0] / nSensors;
    acc[1] += msmnt->acc[1] / nSensors;
    acc[2] += msmnt->acc[2] / nSensors;

    gyr[0] += msmnt->gyr[0] / nSensors;
    gyr[1] += msmnt->gyr[1] / nSensors;
    gyr[2] += msmnt->gyr[2] / nSensors;
  }
}


int main(int argc, char* argv[])
{
  std::ofstream outfile;
  outfile.open("data.txt");

  // number of units used -> remember to set i2cs vector
  unsigned int nSensors = 1;
  // number of measurements to write
  unsigned int measurements = 200000;

  // System setup
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log = hyped::utils::System::getLogger();

  // Initialise array of sensors
  std::vector<Imu *> sensors(nSensors);
  std::vector<ImuData *> imus(nSensors);
  // need to set these values manually
  std::vector<int> i2cs = {66};  // i2c locations of sensors

  assert(nSensors == i2cs.size());
  for (unsigned int i = 0; i < nSensors; ++i)
  {
    Imu * mpu = new Imu(log, i2cs[i], 0x08, 0x00);
    ImuData * imu = new ImuData();
    sensors[i] = mpu;
    imus[i] = imu;
  }

  // write header line
  outfile << "acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z\n";

  for (unsigned int i = 0; i < measurements; i++) {
      NavigationVector acc({0., 0., 0.});
      NavigationVector gyr({0., 0., 0.});
      // get data from IMUs to MPU sensors
      for (unsigned int j = 0; j < nSensors; ++j)
      {
        sensors[j]->getData(imus[j]);
      }
      sensorAverage(acc, gyr, imus);
      outfile << acc[0] << "," << acc[1] << "," << acc[2] << "," << gyr[0] << "," << gyr[1] << "," << gyr[2] << "\n";
  }

  outfile.close();

  // cleanup
  for (unsigned int i = 0; i < nSensors; i++)
  {
    delete imus[i];
  }
}
