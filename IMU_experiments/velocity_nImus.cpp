#include "sensors/mpu9250.hpp"
#include "utils/timer.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include <vector>
#include <iostream>

using hyped::sensors::MPU9250;
using hyped::utils::Logger;
using hyped::utils::Timer;
using hyped::utils::ScopedTimer;
using hyped::utils::concurrent::Thread;
using hyped::data::Imu;
using hyped::data::NavigationType;
using hyped::data::NavigationVector;

void sensorAverage(NavigationVector &acc, NavigationVector &gyr, 
                   std::vector<NavigationVector> &msmnts, int nSensors)
{
  for (NavigationVector &msmnt : msmnts)
  {
    acc[0] += msmnt->acc[0] / nSensors;
    acc[1] += msmnt->acc[1] / nSensors;
    acc[2] += msmnt->acc[2] / nSensors;

    gyr[0] += msmnt->gyr[0] / nSensors;
    gyr[1] += msmnt->gyr[1] / nSensors;
    gyr[2] += msmnt->gyr[2] / nSensors;
  }
}

void sensorAverageGyr(NavigationVector &avg, std::vector<NavigationVector> &msmnts)
{
  int count = 0;
  for (NavigationVector &msmnt : msmnts)
  {

    count++;
  }
  avg[0]/count;
  avg[1]/count;
  avg[2]/count;
}

int main(int argc, char* argv[])
{
  // Some intial parameters
  int nSensors =   2;
  int nQueries = 100;

  // System setup
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log = hyped::utils::System::getLogger();

  // Initialise array of sensors
  std::vector<MPU9250> sensors;
  std::vector<Imu>     imus;
  std::vector<int>     i2cs = {66, 68};  // i2c locations of sensors
  for (int i = 0; i < nSensors; ++i)
  {
    sensors.push_back(MPU9250(log, i2cs[i], 0x08, 0x00));
    imus.push_back(Imu);
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
      // Paralellize querying
      std::vector<Thread> threads;
      // start...
      for (int j = 0; j < nSensors; ++j)
      {
        threads.push_back(&MPU9250::getData, sensors[i], imus[j]);
      }
      // ...stop
      for (Thread &th : threads)
      {
        if (th.joinable()) th.join();
      }
    }
    dt = timer.getMillis();

    // Determine velocity
    {
      ScopedTimer scope_timer(&timer);

      sensorAverage(&acc, &gyr, &imus);
      vel[0] += acc[0]*dt;
      vel[1] += acc[1]*dt;
      vel[2] += acc[2]*dt;
    }
    dt = timer.getMillis();


    // Print
    log.DBG("TEST-mpu9250", "velocity readings x: %f m/s^2, y: %f m/s^2, z: %f m/s^2\tblind time: %f\n", 
                                                                     vel[0], vel[1], vel[2], dt);

  }
}
