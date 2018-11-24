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

float compute_velocity(long int time_difference, float previous_velocity, float acceleration_1, float acceleration_2) {
  return (previous_velocity + (time_difference / 1000.0) * (acceleration_2 - acceleration_1)/2.0);
}

float compute_distance_travelled(float velocity, long int time_difference) {
  // distance in metre
  return velocity * (time_difference / 1000.0)
}

std::vector<float> compute_velocities(int number_of_measures, std::vector<long int> time_stamps, std::vector<float> accelerations) {
  std::vector<float> velocities(number_of_measures);
  velocities[0] = 0.0;
  for (int i=1; i < number_of_measures; i++) {
    velocities[i] = velocities[i-1] + (((time_stamps[i] - time_stamps[i-1]) * 1.0) / 1000.0) * (accelerations[i] - accelerations[i-1])/2;
  }
  return velocities;
}

float average(int number_of_measures, std::vector<float> values) {
  float values_sum = 0.0;
  for (int i=0; i < number_of_measures; i++) {
    values_sum += values[i];
  }

  return values_sum/number_of_measures;

}

int main(int argc, char* argv[])
{
  hyped::utils::System::parseArgs(argc, argv);
  Logger& log = hyped::utils::System::getLogger();
  MPU9250 mpu9250(log, 66, 0x08, 0x00);
  Imu imu;

  int number_of_measures = 100;

  std::vector<float> imu_acc_x(number_of_measures);
  std::vector<float> imu_acc_y(number_of_measures);
  std::vector<float> imu_acc_z(number_of_measures);

  std::vector<long int> time_stamps(number_of_measures);

  Timer timer;
  for (int i=0; i < number_of_measures; i++) {
    {
      ScopedTimer scope_timer(&timer);
      mpu9250.getData(&imu);
      imu_acc_x[i] = imu.acc[0];
      imu_acc_y[i] = imu.acc[1];
      imu_acc_z[i] = imu.acc[2];
    }
    time_stamps[i] = timer.getMillis();
  }

  std::vector<float> imu_v_x = compute_velocity(number_of_measures, time_stamps, imu_acc_x);
  std::vector<float> imu_v_y = compute_velocity(number_of_measures, time_stamps, imu_acc_y);
  std::vector<float> imu_v_z = compute_velocity(number_of_measures, time_stamps, imu_acc_z);

  float imu_acc_x_avg = average(number_of_measures, imu_acc_x);
  float imu_acc_y_avg = average(number_of_measures, imu_acc_y);
  float imu_acc_z_avg = average(number_of_measures, imu_acc_z);

  std::cout << "IMU accelerometer X avg: " << imu_acc_x_avg << std::endl;
  std::cout << "IMU accelerometer Y avg: " << imu_acc_y_avg << std::endl;
  std::cout << "IMU accelerometer Z avg: " << imu_acc_z_avg << std::endl;

  std::cout << "IMU velocity X avg: " << imu_v_x[number_of_measures - 1] << std::endl;
  std::cout << "IMU velocity Y avg: " << imu_v_y[number_of_measures - 1] << std::endl;
  std::cout << "IMU velocity Z avg: " << imu_v_z[number_of_measures - 1] << std::endl;
}
