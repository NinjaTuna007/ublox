#!/usr/bin/env python3
"""GNSS over Teensy virtual serial: NMEA -> NavSatFix + SMARC topics."""

import math
import threading
import time
from typing import Optional

import rclpy
from geographic_msgs.msg import GeoPoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32

try:
    import serial
except ImportError as exc:
    raise ImportError(
        'python3-serial is required for nmea_serial_gnss. '
        'Install with: sudo apt install python3-serial'
    ) from exc


def _nmea_checksum_ok(line: str) -> bool:
    star = line.find('*')
    if star < 0 or star + 3 > len(line):
        return False
    body = line[1:star]
    try:
        expected = int(line[star + 1:star + 3], 16)
    except ValueError:
        return False
    actual = 0
    for ch in body:
        actual ^= ord(ch)
    return actual == expected


def _ddmm_to_deg(value: str, hemisphere: str) -> float:
    if not value or not hemisphere:
        return float('nan')
    dot = value.find('.')
    if dot < 2:
        return float('nan')
    degrees = int(value[: dot - 2])
    minutes = float(value[dot - 2:])
    signed = degrees + minutes / 60.0
    if hemisphere in ('S', 'W'):
        signed = -signed
    return signed


def _gga_fix_status(quality: int) -> int:
    if quality <= 0:
        return NavSatStatus.STATUS_NO_FIX
    if quality == 1:
        return NavSatStatus.STATUS_FIX
    if quality == 2:
        return NavSatStatus.STATUS_SBAS_FIX
    if quality in (4, 5):
        return NavSatStatus.STATUS_GBAS_FIX
    return NavSatStatus.STATUS_FIX


class NmeaSerialGnss(Node):
    """Read NMEA from a serial port and publish fix + SMARC topics."""

    def __init__(self):
        super().__init__('nmea_serial_gnss')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('frame_id', 'stick_1/base_link')
        self.declare_parameter('fix_topic', 'ublox_gps_node/fix')
        self.declare_parameter('reconnect_delay_sec', 2.0)

        self._port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('serial_baud').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._fix_topic = self.get_parameter('fix_topic').value
        self._reconnect_delay = float(
            self.get_parameter('reconnect_delay_sec').value)

        gps_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.fix_pub = self.create_publisher(NavSatFix, self._fix_topic, gps_qos)
        self.latlon_pub = self.create_publisher(GeoPoint, 'smarc/latlon', 10)
        self.altitude_pub = self.create_publisher(Float32, 'smarc/altitude', 10)
        self.speed_pub = self.create_publisher(Float32, 'smarc/speed', 10)
        self.heading_pub = self.create_publisher(Float32, 'smarc/heading', 10)

        self._serial: Optional[serial.Serial] = None
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.get_logger().info(
            f'NMEA serial GNSS on {self._port} @ {self._baud} '
            f'(RTK corrections require a UBX/RTCM path, not NMEA-only)')

    def _open_serial(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                timeout=1.0,
            )
            self.get_logger().info(f'Opened serial port {self._port}')
            return True
        except serial.SerialException as exc:
            self.get_logger().warn(
                f'Cannot open {self._port}: {exc} — retrying in '
                f'{self._reconnect_delay:.0f}s')
            self._serial = None
            return False

    def _read_loop(self):
        while not self._stop.is_set():
            if self._serial is None:
                if not self._open_serial():
                    time.sleep(self._reconnect_delay)
                    continue

            try:
                raw = self._serial.readline()
            except serial.SerialException as exc:
                self.get_logger().warn(f'Serial read error: {exc}')
                if self._serial is not None:
                    self._serial.close()
                self._serial = None
                time.sleep(self._reconnect_delay)
                continue

            if not raw:
                continue

            try:
                line = raw.decode('ascii', errors='ignore').strip()
            except Exception:
                continue

            if not line.startswith('$'):
                continue
            if '*' in line and not _nmea_checksum_ok(line):
                continue

            if line[3:6] == 'GGA' or line[4:7] == 'GGA':
                self._handle_gga(line)
            elif line[3:6] == 'RMC' or line[4:7] == 'RMC':
                self._handle_rmc(line)
            elif line[3:6] == 'VTG' or line[4:7] == 'VTG':
                self._handle_vtg(line)

    def _publish_fix(
        self,
        lat: float,
        lon: float,
        alt_msl: float,
        quality: int,
        num_sats: int,
    ):
        if math.isnan(lat) or math.isnan(lon):
            return

        stamp = self.get_clock().now().to_msg()

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self._frame_id
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt_msl
        fix.status.status = _gga_fix_status(quality)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(fix)

        latlon = GeoPoint()
        latlon.latitude = lat
        latlon.longitude = lon
        latlon.altitude = alt_msl
        self.latlon_pub.publish(latlon)

        alt_msg = Float32()
        alt_msg.data = float(alt_msl)
        self.altitude_pub.publish(alt_msg)

        if quality <= 0:
            return

        self.get_logger().debug(
            f'fix q={quality} sats={num_sats} '
            f'lat={lat:.6f} lon={lon:.6f} alt={alt_msl:.1f}',
            throttle_duration_sec=5.0)

    def _publish_motion(self, speed_mps: float, heading_deg: float):
        if speed_mps < 0.0 or math.isnan(speed_mps):
            return

        speed_msg = Float32()
        speed_msg.data = float(speed_mps)
        self.speed_pub.publish(speed_msg)

        if math.isnan(heading_deg):
            heading_rad = 0.0
        else:
            # NMEA course: degrees clockwise from true north.
            heading_rad = math.radians(heading_deg)
        heading_msg = Float32()
        heading_msg.data = float(heading_rad)
        self.heading_pub.publish(heading_msg)

    def _handle_gga(self, line: str):
        # $GNGGA,time,lat,N,lon,E,quality,num_sats,hdop,alt,M,geoid,M,,*cs
        parts = line.split(',')
        if len(parts) < 10:
            return
        try:
            quality = int(parts[6] or '0')
        except ValueError:
            quality = 0
        try:
            num_sats = int(parts[7] or '0')
        except ValueError:
            num_sats = 0

        lat = _ddmm_to_deg(parts[2], parts[3])
        lon = _ddmm_to_deg(parts[4], parts[5])
        try:
            alt_msl = float(parts[9]) if parts[9] else float('nan')
        except ValueError:
            alt_msl = float('nan')

        self._publish_fix(lat, lon, alt_msl, quality, num_sats)

    def _handle_rmc(self, line: str):
        # $GNRMC,time,status,lat,N,lon,E,speed_kn,course_deg,date,...*cs
        parts = line.split(',')
        if len(parts) < 9:
            return
        if parts[2] != 'A':
            return

        speed_mps = float('nan')
        heading_deg = float('nan')
        try:
            if parts[7]:
                speed_mps = float(parts[7]) * 0.514444
        except ValueError:
            pass
        try:
            if parts[8]:
                heading_deg = float(parts[8])
        except ValueError:
            pass

        self._publish_motion(speed_mps, heading_deg)

    def _handle_vtg(self, line: str):
        # $GNVTG,course,T,,M,speed_kn,N,speed_kmh,K,mode*cs
        parts = line.split(',')
        if len(parts) < 8:
            return

        heading_deg = float('nan')
        speed_mps = float('nan')
        try:
            if parts[1]:
                heading_deg = float(parts[1])
        except ValueError:
            pass
        try:
            if parts[5]:
                speed_mps = float(parts[5]) * 0.514444
            elif parts[7]:
                speed_mps = float(parts[7]) / 3.6
        except ValueError:
            pass

        self._publish_motion(speed_mps, heading_deg)

    def destroy_node(self):
        self._stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=2.0)
        if self._serial is not None:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NmeaSerialGnss()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
