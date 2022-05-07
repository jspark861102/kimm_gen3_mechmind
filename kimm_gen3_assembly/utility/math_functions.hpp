
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

static Eigen::Matrix3d skm(Eigen::Vector3d x)
  {
    Eigen::Matrix3d Skew_temp1(3, 3);
    Skew_temp1.setZero();
    Skew_temp1(0, 1) = -x(2);
    Skew_temp1(0, 2) = x(1);
    Skew_temp1(1, 0) = x(2);
    Skew_temp1(1, 2) = -x(0);
    Skew_temp1(2, 0) = -x(1);
    Skew_temp1(2, 1) = x(0);
    return Skew_temp1;
  }

  static double cubic(double time,    ///< Current time
                      double time_0,  ///< Start time
                      double time_f,  ///< End time
                      double x_0,     ///< Start state
                      double x_f,     ///< End state
                      double x_dot_0, ///< Start state dot
                      double x_dot_f  ///< End state dot
  )
  {
    double x_t;

    if (time < time_0)
    {
      x_t = x_0;
    }
    else if (time > time_f)
    {
      x_t = x_f;
    }
    else
    {
      double elapsed_time = time - time_0;
      double total_time = time_f - time_0;
      double total_time2 = total_time * total_time;  // pow(t,2)
      double total_time3 = total_time2 * total_time; // pow(t,3)
      double total_x = x_f - x_0;

      x_t = x_0 + x_dot_0 * elapsed_time

            + (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

            + (-2 * total_x / total_time3 +
               (x_dot_0 + x_dot_f) / total_time2) *
                  elapsed_time * elapsed_time * elapsed_time;
    }

    return x_t;
  }