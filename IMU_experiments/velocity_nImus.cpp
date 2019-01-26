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
#include <unistd.h>
#include <math.h>
#include <cstdlib> 
#include <math.h>

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


float norm(NavigationVector &v)
{
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}


NavigationVector computeAvgAcc(unsigned int nSensors, std::vector<Imu *> &sensors,
                                  std::vector<ImuData *> &imus, unsigned int measurements)
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
  unsigned int nQueries = 200000;
  double last_time = 0.0;

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

  /*
  if (writeFile) {
      for (int i = 0; i < 200000; i++) {
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
  }

  if (writeFile) {
    outfile.close();
  }

  return 1;*/

  // compute gravity acceleration given current orientation
  // get 100 measurements
  unsigned int grav_measurements = 1000;
  NavigationVector acc_gravity = computeAvgAcc(nSensors, sensors, imus, grav_measurements);
  // recompute gravity until the value seems reasonable
  //float absSum = norm(acc_gravity);
  // should be 9.81
  /*
  while (absSum < 9.76 || absSum > 9.86)
  {
      log.INFO("Gravity Acceleration", "Sum of absolute values is %f which is not plausible -> recompute gravity acceleration", absSum);
      if (writeFile) {
        outfile << "Gravity Acceleration: Sum of absolute values is " << absSum << " which is not plausible -> recompute gravity acceleration\n";
      }
      acc_gravity = computeAvgAcc(nSensors, sensors, imus, grav_measurements);
      absSum = norm(acc_gravity);
  }
  */
  

  log.INFO("Gravity Acceleration", "x: %f m/s^2, y: %f m/s^2, z: %f m/s^2",
          acc_gravity[0], acc_gravity[1], acc_gravity[2]);
  // Outfile headers
  if (writeFile) {
    outfile << "Gravity Acceleration: x: " << acc_gravity[0] << " m/s^2, y: " << acc_gravity[1] << " m/s^2, z: " << acc_gravity[2] << " m/s^2\n";
    outfile << "ax\tay\taz\tvx\tvy\tvz\tsx\tsy\tsz\tblind time\n\n";
  }

  std::cout << "Calibration completed, measuring." << std::endl;

  // Perform specified number of measurements
  Timer timer;
  double dt;
  // get initial measurements
  NavigationVector acc_prev({0., 0., 0.});
  NavigationVector gyr_prev({0., 0., 0.});
  sensorAverage(acc_prev, gyr_prev, imus);
  NavigationVector vel_prev({0., 0., 0.}); 
  NavigationVector vel({0., 0., 0.});      // assume starting at rest
  NavigationVector pos({0., 0., 0.});      // define origin
  unsigned int query = 0;
  for (unsigned int i = 0; i < nQueries; i++) {
    // Measure acceleration
    NavigationVector acc_new({0., 0., 0.});
    NavigationVector gyr_new({0., 0., 0.});
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
    dt = (current_time - last_time)/1000.0;
    last_time = current_time;

    // Determine velocity and position
    {
      ScopedTimer scope_timer(&timer);

      sensorAverage(acc_new, gyr_new, imus);
      for (unsigned int j = 0; j < 3; j++)
      {
          acc_new[j] -= acc_gravity[j];
          vel[j] += (acc_new[j] + acc_prev[j])/2 * dt;
          pos[j] += (vel[j] + vel_prev[j])/2 * dt;

          // update stored values
          acc_prev[j] = acc_new[j];
          vel_prev[j] = vel[j];
      }

      
    }


    // Print
    log.INFO("IMU data", "acceleration readings x: %f m/s^2, y: %f m/s^2, z: %f m/s^2\tblind time: %f",
                                                                     acc_new[0], acc_new[1], acc_new[2], dt);
    log.INFO("Navigation computation", "velocity computed x: %f m/s, y: %f m/s, z: %f m/s",
                                                                     vel[0], vel[1], vel[2]);
    log.INFO("Navigation computation", "position computed x: %f m, y: %f m, z: %f m",
                                                                     pos[0], pos[1], pos[2]);
    if (writeFile) {
      outfile << acc_new[0] << "\t" << acc_new[1] << "\t" << acc_new[2] << "\t" 
              << vel[0] << "\t" << vel[1] << "\t" << vel[2] << "\t" 
              << pos[0] << "\t" << pos[1] << "\t" << pos[2] << "\t"
              << dt     << "\n";
    }

    query++;
  }

  std::cout << "Finished measurements." << std::endl;

  if (writeFile) {
    outfile.close();
  }
  // cleanup
  for (unsigned int i = 0; i < nSensors; i++)
  {
    delete imus[i];
  }

}
