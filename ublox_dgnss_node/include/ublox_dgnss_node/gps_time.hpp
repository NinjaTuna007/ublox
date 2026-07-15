// Copyright 2026 Stick stack contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef UBLOX_DGNSS_NODE__GPS_TIME_HPP_
#define UBLOX_DGNSS_NODE__GPS_TIME_HPP_

#include <cstdint>
#include <ctime>

namespace ublox_dgnss
{

inline constexpr int64_t kGpsWeekMs = 604800000LL;

inline int64_t utc_fields_to_unix_ns(
  uint16_t year, uint8_t month, uint8_t day,
  uint8_t hour, uint8_t min, uint8_t sec, int32_t nano)
{
  std::tm tm = {};
  tm.tm_year = static_cast<int>(year) - 1900;
  tm.tm_mon = static_cast<int>(month) - 1;
  tm.tm_mday = static_cast<int>(day);
  tm.tm_hour = static_cast<int>(hour);
  tm.tm_min = static_cast<int>(min);
  tm.tm_sec = static_cast<int>(sec);
  tm.tm_isdst = 0;

  const time_t seconds = timegm(&tm);
  if (seconds < 0) {
    return 0;
  }

  return static_cast<int64_t>(seconds) * 1000000000LL + static_cast<int64_t>(nano);
}

inline int64_t itow_delta_ms(uint32_t anchor_itow_ms, uint32_t itow_ms)
{
  int64_t delta = static_cast<int64_t>(itow_ms) - static_cast<int64_t>(anchor_itow_ms);
  if (delta > kGpsWeekMs / 2) {
    delta -= kGpsWeekMs;
  } else if (delta < -kGpsWeekMs / 2) {
    delta += kGpsWeekMs;
  }
  return delta;
}

struct GpsTimeAnchor
{
  bool valid = false;
  uint32_t itow_ms = 0;
  int64_t unix_ns = 0;
};

inline int64_t stamp_ns_from_itow(const GpsTimeAnchor & anchor, uint32_t itow_ms)
{
  if (!anchor.valid) {
    return 0;
  }
  const int64_t delta_ms = itow_delta_ms(anchor.itow_ms, itow_ms);
  return anchor.unix_ns + (delta_ms * 1000000LL);
}

}  // namespace ublox_dgnss

#endif  // UBLOX_DGNSS_NODE__GPS_TIME_HPP_
