#include "sensors/mpu9250.hpp"
#include "utils/timer.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <unistd.h>
#include <cstdlib> 

using hyped::sensors::MPU9250;
using hyped::utils::Logger;
using hyped::utils::Timer;
using hyped::utils::ScopedTimer;
using hyped::utils::concurrent::Thread;
using hyped::data::Imu;
using hyped::data::NavigationType;
using hyped::data::NavigationVector;

void sensorAverage(NavigationVector &acc, NavigationVector &gyr, std::vector<Imu *> & msmnts)
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


float absoluteSum(NavigationVector &v)
{
    float absSum = 0.0;
    for (unsigned int i = 0; i < 3; i++) {
        absSum += abs(v[i]);
    }
    return absSum;
}


NavigationVector computeAvgAcc(unsigned int nSensors, std::vector<MPU9250 *> &sensors,
                                  std::vector<Imu *> &imus, unsigned int measurements)
{
    std::vector<NavigationVector> accelerations(measurements);
    for (unsigned int i = 0; i < measurements; i++)
      {
          NavigationVector acc({0., 0., 0.});
          NavigationVector gyr({0., 0., 0.});
          // get data from IMUs to MPU sensors
          for (unsigned int j = 0; j < nSensors; ++j)
          {
            sensors[j]->getData(imus[j]);
          }
          sensorAverage(acc, gyr, imus);
          accelerations[i] = acc;

          sleep(1.0/float(measurements));
      }

      // compute the average of 100 accs for gravity
      NavigationVector acc_gravity({0., 0., 0.});
      for (unsigned int i = 0; i < measurements; i++)
      {
          for (int j = 0; j < 3; j++)
          {
              acc_gravity[j] += accelerations[i][j]/float(measurements);
          }
      }
      return acc_gravity;
}


int main(int argc, char* argv[])
{
  bool writeFile = true;
  std::ofstream outfile;
  if (writeFile) {
    outfile.open("data.txt");
  }

  // Some intial parameters
  // number of units used -> remember to set i2cs vector
  unsigned int nSensors = 1;
  // number of iterations
  unsigned int nQueries = 10000;
  double last_time = 0.0;

  // System setup
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log = hyped::utils::System::getLogger();

  // Initialise array of sensors
  std::vector<MPU9250 *> sensors(nSensors);
  std::vector<Imu *> imus(nSensors);
  // need to set these values manually
  std::vector<int> i2cs = {66};  // i2c locations of sensors

  assert(nSensors == i2cs.size());
  for (unsigned int i = 0; i < nSensors; ++i)
  {
    MPU9250 * mpu = new MPU9250(log, i2cs[i], 0x08, 0x00);
    Imu * imu = new Imu();
    sensors[i] = mpu;
    imus[i] = imu;
  }

  // compute gravity acceleration given current orientation
  // get 100 measurements
  unsigned int grav_measurements = 100;
  NavigationVector acc_gravity = computeAvgAcc(nSensors, sensors, imus, grav_measurements);
  // recompute gravity until the value seems reasonable
  float absSum = absoluteSum(acc_gravity);
  // should be 9.81
  while (absSum < 9.76 || absSum > 9.86)
  {
      log.INFO("Gravity Acceleration", "Sum of absolute values is %f which is not plausible -> recompute gravity acceleration", absSum);
      if (writeFile) {
        outfile << "Gravity Acceleration: Sum of absolute values is " << absSum << " which is not plausible -> recompute gravity acceleration\n";
      }
      sleep(0.5);
      acc_gravity = computeAvgAcc(nSensors, sensors, imus, grav_measurements);
  }
  

  log.INFO("Gravity Acceleration", "x: %f m/s^2, y: %f m/s^2, z: %f m/s^2",
          acc_gravity[0], acc_gravity[1], acc_gravity[2]);
  if (writeFile) {
    outfile << "Gravity Acceleration: x: " << acc_gravity[0] << " m/s^2, y: " << acc_gravity[1] << " m/s^2, z: " << acc_gravity[2] << " m/s^2\n\n";
  }

  // Perform specified number of measurements
  Timer timer;
  double dt;
  // velocity in m/s
  NavigationVector vel({0., 0., 0.});
  // position in m starting at (0,0,0)
  NavigationVector pos({0., 0., 0.});
  unsigned int query = 0;
  for (unsigned int i = 0; i < nQueries; i++) {
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
    last_time = current_time;

    // Determine velocity and position
    {
      ScopedTimer scope_timer(&timer);

      sensorAverage(acc, gyr, imus);
      for (unsigned int j = 0; j < 3; j++)
      {
          acc[j] -= acc_gravity[j];
          vel[j] += acc[j] * dt;
          pos[j] += vel[j] * dt;
      }
    }


    // Print
    log.INFO("IMU data", "acceleration readings x: %f m/s^2, y: %f m/s^2, z: %f m/s^2\tblind time: %f",
                                                                     acc[0], acc[1], acc[2], dt);
    if (writeFile) {
      outfile << "IMU data: acceleration readings x: " << acc[0] << " m/s^2, y: " << acc[1] << " m/s^2, z: " << acc[2] << " m/s^2\tblind time: " << dt << "\n";
    }
    log.INFO("Navigation computation", "velocity computed x: %f m/s, y: %f m/s, z: %f m/s",
                                                                     vel[0], vel[1], vel[2]);
    if (writeFile) {
      outfile << "Navigation computation: velocity computed x: " << vel[0] << " m/s, y: " << vel[1] << " m/s, z: " << vel[2] << " m/s\n";
    }
    log.INFO("Navigation computation", "position computed x: %f m, y: %f m, z: %f m",
                                                                     pos[0], pos[1], pos[2]);
    if (writeFile) {
      outfile << "Navigation computation: position computed x: " << pos[0] << " m, y: " << pos[1] << " m, z: " << pos[2] << " m\n\n";
    }

    query++;
  }

  if (writeFile) {
    outfile.close();
  }
  // cleanup
  for (unsigned int i = 0; i < nSensors; i++)
  {
    delete imus[i];
  }

}
