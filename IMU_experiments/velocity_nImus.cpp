#include "sensors/mpu9250.hpp"
#include "utils/timer.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include <vector>
#include <iostream>
#include <assert.h>

using hyped::sensors::MPU9250;
using hyped::utils::Logger;
using hyped::utils::Timer;
using hyped::utils::ScopedTimer;
using hyped::utils::concurrent::Thread;
using hyped::data::Imu;
using hyped::data::NavigationType;
using hyped::data::NavigationVector;

void sensorAverage(NavigationVector &acc, NavigationVector &gyr, 
                   std::vector<Imu *> & msmnts)
{
  float nSensors = float(msmnts.size());
  for (Imu *msmnt : msmnts)
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
  // Some intial parameters
  unsigned int nSensors =   1;
  int nQueries = 100;
  double last_time = 0.0;

  // System setup
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log = hyped::utils::System::getLogger();

  // Initialise array of sensors
  std::vector<MPU9250 *> sensors(nSensors);
  std::vector<Imu *>     imus(nSensors);
  std::vector<int>     i2cs = {66};  // i2c locations of sensors

  assert(nSensors == i2cs.size());
  for (unsigned int i = 0; i < nSensors; ++i)
  {
    MPU9250 * mpu = new MPU9250(log, i2cs[i], 0x08, 0x00);
    Imu * imu = new Imu();
    sensors[i] = mpu;
    imus[i] = imu;
  }

  // Perform specified number of measurements
  Timer timer;
  double dt;
  NavigationVector vel({0., 0., 0.});
  for (int i = 0; i < nQueries; i++) {
    // Measure acceleration
    NavigationVector acc({0., 0., 0.});
    NavigationVector gyr({0., 0., 0.});
    // New scope to generate dt
    {
      ScopedTimer scope_timer(&timer);

      // get data from IMUs to MPU sensors
      for (unsigned int j = 0; j < nSensors; ++j)
      {
        sensors[j]->getData(imus[j]);
      }
    }
    // time stamp in seconds
    double current_time = timer.getMillis();
    dt = (current_time - last_time)/ 1000.0;
    std::cout << dt << std::endl;
    last_time = current_time;

    // Determine velocity
    {
      ScopedTimer scope_timer(&timer);

      sensorAverage(acc, gyr, imus);
      vel[0] += acc[0]*dt;
      vel[1] += acc[1]*dt;
      vel[2] += acc[2]*dt;
    }


    // Print
    log.INFO("TEST-mpu9250", "velocity readings x: %f m/s^2, y: %f m/s^2, z: %f m/s^2\tblind time: %f\n", 
                                                                     vel[0], vel[1], vel[2], dt);

  }

  // cleanup
  for (unsigned int i = 0; i < nSensors; i++)
  {
    delete sensors[i];
    delete imus[i];
  }

}
