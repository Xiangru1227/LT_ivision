#pragma once

template <typename T>
T RadianToDegree(T radian_angle) {
  return 57.29577951308 * radian_angle;
}

template <typename T>
T DegreeToRadian(T degree_angle) {
  return 0.01745329251994 * degree_angle;
}