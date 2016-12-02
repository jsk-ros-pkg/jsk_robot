/*
 * spline_test.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <iostream>
#include <gtest/gtest.h>
#include <pr2_base_trajectory_action/pr2_base_trajectory_action_controller.h>

using namespace std;
using namespace pr2_base_trajectory_action;

TEST(TestSuite, testSpline)
{
  Motion mx0(0.0, 0.23599999999283272);
  Motion mx1(1.0, 0.0);
  Motion my0(0.0, 0.0);
  Motion my1(0.0, 0.0);
  Motion myaw0(0.0, 0.0);
  Motion myaw1(0.0, 0.0);
  double duration = 4.237288128;
  CubicSpline spx(mx0, mx1, duration);
  CubicSpline spy(my0, my1, duration);
  CubicSpline spyaw(myaw0, myaw1, duration);

  Trajectory::Segment s;
  s.start_time = 0.0;
  s.duration = duration;
  s.splines[0] = spx;
  s.splines[1] = spy;
  s.splines[2] = spyaw;

  cout << "x: " << endl;
  for (int i = 0; i < 6; ++i)
    cout << "  " << i << ": " << spx.coefficients[i] << endl;
  std::vector<double> spx_true(6, 0.0);
  spx_true[1] = 0.236;
  spx_true[2] = 0.055696;
  spx_true[3] = -0.0131443;
  for(int i = 0; i < 6; ++i)
    ASSERT_NEAR(spx.coefficients[i], spx_true[i], 0.001);

  cout << "y: " << endl;
  for (int i = 0; i < 6; ++i)
    cout << "  " << i << ": " << spy.coefficients[i] << endl;
  for(int i = 0; i < 6; ++i)
    ASSERT_NEAR(spy.coefficients[i], 0.0, 0.001);

  cout << "yaw: " << endl;
  for (int i = 0; i < 6; ++i)
    cout << "  " << i << ": " << spyaw.coefficients[i] << endl;
  for(int i = 0; i < 6; ++i)
    ASSERT_NEAR(spyaw.coefficients[i], 0.0, 0.001);

  std::vector<double> sample_true;
  sample_true.push_back(0.0);
  sample_true.push_back(0.0754577);
  sample_true.push_back(0.158811);
  sample_true.push_back(0.247932);
  sample_true.push_back(0.340689);
  sample_true.push_back(0.434954);
  sample_true.push_back(0.528598);
  sample_true.push_back(0.61949);
  sample_true.push_back(0.705503);
  sample_true.push_back(0.784505);
  sample_true.push_back(0.854369);
  sample_true.push_back(0.912964);
  sample_true.push_back(0.958162);
  sample_true.push_back(0.987832);
  sample_true.push_back(0.999846);

  int i = 0;
  for (double t = 0.0; t < duration; t += 0.3) {
    Motion x,y,yaw;
    spx.sampleWithTimeBounds(duration, t, x);
    spy.sampleWithTimeBounds(duration, t, y);
    spyaw.sampleWithTimeBounds(duration, t, yaw);
    ASSERT_NEAR(x.position, sample_true[i], 0.001);
    cout << "t: " << t << ", x: " << x.position << " y: " << y.position << " yaw: " << yaw.position << endl;
    ++i;
  }

  i = 0;
  for (double t = 0.0; t < duration; t += 0.3) {
    BaseMotion b;
    s.sample(t, b);
    cout << "t: " << t << ", " << b << endl;
    ASSERT_NEAR(b.x.position, sample_true[i], 0.001);
    ++i;
  }
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
