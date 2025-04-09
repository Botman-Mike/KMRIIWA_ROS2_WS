#!/usr/bin/env python3

import struct
import ctypes
import numpy as np
import math
import sys
from sensor_msgs.msg import PointField, PointCloud2

# Class for creating PointCloud2 messages from LaserScan messages. Parts of the code is inspired by out-of-date packages for assembling laser scans such as laser_geometry.

class LaserToPointcloud():

    LASER_SCAN_INVALID   = -1.0
    LASER_SCAN_MIN_RANGE = -2.0
    LASER_SCAN_MAX_RANGE = -3.0

    class ChannelOption:
        NONE      = 0x00 # Enable no channels
        INTENSITY = 0x01 # Enable "intensities" channel
        INDEX     = 0x02 # Enable "index"       channel
        DISTANCE  = 0x04 # Enable "distances"   channel
        TIMESTAMP = 0x08 # Enable "stamps"      channel
        VIEWPOINT = 0x10 # Enable "viewpoint"   channel
        DEFAULT   = (INTENSITY | INDEX)

    def __init__(self):
        self.__angle_min = 0.0
        self.__angle_max = 0.0
        self.__cos_sin_map = np.array([[]])

        self._DATATYPES = {}
        self._DATATYPES[PointField.INT8]    = ('b', 1)
        self._DATATYPES[PointField.UINT8]   = ('B', 1)
        self._DATATYPES[PointField.INT16]   = ('h', 2)
        self._DATATYPES[PointField.UINT16]  = ('H', 2)
        self._DATATYPES[PointField.INT32]   = ('i', 4)
        self._DATATYPES[PointField.UINT32]  = ('I', 4)
        self._DATATYPES[PointField.FLOAT32] = ('f', 4)
        self._DATATYPES[PointField.FLOAT64] = ('d', 8)

    def projectLaser(self, scan_in,
            range_cutoff=-1.0, channel_options=ChannelOption.DEFAULT):
        """
        Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud2.
        Project a single laser scan from a linear array into a 3D
        point cloud. The generated cloud will be in the same frame
        as the original laser scan.
        Keyword arguments:
        scan_in -- The input laser scan.
        range_cutoff -- An additional range cutoff which can be
            applied which is more limiting than max_range in the scan
            (default -1.0).
        channel_options -- An OR'd set of channels to include.
        """
        return self.__projectLaser(scan_in, range_cutoff, channel_options)


    def __projectLaser(self, scan_in, range_cutoff, channel_options):
        N = len(scan_in.ranges)

        ranges = np.array(scan_in.ranges)

        if (self.__cos_sin_map.shape[1] != N or
            self.__angle_min != scan_in.angle_min or
            self.__angle_max != scan_in.angle_max):

            self.__angle_min = scan_in.angle_min
            self.__angle_max = scan_in.angle_max
            
            angles = scan_in.angle_min + np.arange(N) * scan_in.angle_increment
            self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])

        output = ranges * self.__cos_sin_map

        # Set the output cloud accordingly
        cloud_out = PointCloud2()

        fields = [PointField() for _ in range(3)]

        fields[0].name = "x"
        fields[0].offset = 0
        fields[0].datatype = PointField.FLOAT32
        fields[0].count = 1

        fields[1].name = "y"
        fields[1].offset = 4
        fields[1].datatype = PointField.FLOAT32
        fields[1].count = 1

        fields[2].name = "z"
        fields[2].offset = 8
        fields[2].datatype = PointField.FLOAT32
        fields[2].count = 1

        idx_intensity = idx_index = idx_distance =  idx_timestamp = -1
        idx_vpx = idx_vpy = idx_vpz = -1

        offset = 12

        if (channel_options & self.ChannelOption.INTENSITY and
            len(scan_in.intensities) > 0):
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "intensity"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_intensity = field_size

        if channel_options & self.ChannelOption.INDEX:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "index"
            fields[field_size].datatype = PointField.INT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_index = field_size

        if channel_options & self.ChannelOption.DISTANCE:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "distances"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_distance = field_size

        if channel_options & self.ChannelOption.TIMESTAMP:
            field_size = len(fields)
            fields.append(PointField())
            fields[field_size].name = "stamps"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_timestamp = field_size

        if channel_options & self.ChannelOption.VIEWPOINT:
            field_size = len(fields)
            fields.extend([PointField() for _ in range(3)])
            fields[field_size].name = "vp_x"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpx = field_size
            field_size += 1

            fields[field_size].name = "vp_y"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpy = field_size
            field_size += 1

            fields[field_size].name = "vp_z"
            fields[field_size].datatype = PointField.FLOAT32
            fields[field_size].offset = offset
            fields[field_size].count = 1
            offset += 4
            idx_vpz = field_size

        if range_cutoff < 0:
            range_cutoff = scan_in.range_max
        else:
            range_cutoff = min(range_cutoff, scan_in.range_max)

        points = []
        for i in range(N):
            ri = scan_in.ranges[i]
            if ri < range_cutoff and ri >= scan_in.range_min:
                point = output[:, i].tolist()
                point.append(0)

                if idx_intensity != -1:
                    point.append(scan_in.intensities[i])

                if idx_index != -1:
                    point.append(i)

                if idx_distance != -1:
                    point.append(scan_in.ranges[i])

                if idx_timestamp != -1:
                    point.append(i * scan_in.time_increment)

                if idx_vpx != -1 and idx_vpy != -1 and idx_vpz != -1:
                    point.extend([0 for _ in range(3)])

                points.append(point)

        cloud_out = self.create_cloud(scan_in.header, fields, points)

        return cloud_out


    def concatenate_clouds(self, cloud1, cloud2):
        '''
        Concatenates two PointCloud2 messages in the same frame.
        @param cloud1: Cloud expressed in the goal frame.
        @type cloud1: sensor_msgs.msg.PointCloud2
        @param cloud2: Cloud expressed in the goal frame.
        @type cloud2: sensor_msgs.msg.PointCloud2
        '''
        points_1 = self.read_points(cloud1)
        points_2 = self.read_points(cloud2)

        points_concatenated = []

        for point in points_1:
            points_concatenated.append(point)

        for point in points_2:
            points_concatenated.append(point)

        header = cloud1.header

        fields = cloud1.fields
        # The header doesnt really matter, as it is changed later.
        concatenated_cloud = self.create_cloud(cloud1.header, fields, points_concatenated)
        return concatenated_cloud


    def create_cloud(self, header, fields, points):
        """
        Create a sensor_msgs.msg.PointCloud2 message.
        @param header: The point cloud header.
        @type  header: std_msgs.msg.Header
        @param fields: The point cloud fields.
        @type  fields: iterable of sensor_msgs.msg.PointField
        @param points: The point cloud points.
        @type  points: list of iterables, i.e. one iterable for each point, with the
                       elements of each iterable being the values of the fields for 
                       that point (in the same order as the fields parameter)
                       [(x,y,z,w,no.),---,]
        @return: The point cloud.
        @rtype:  sensor_msgs.msg.PointCloud2
        """

        cloud_struct = struct.Struct(self._get_struct_fmt(False, fields))

        buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

        point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
        offset = 0
        for p in points:
            pack_into(buff, offset, *p)
            offset += point_step

        return PointCloud2(header=header,
                           height=1,
                           width=len(points),
                           is_dense=False,
                           is_bigendian=False,
                           fields=fields,
                           point_step=cloud_struct.size,
                           row_step=cloud_struct.size * len(points),
                           data=buff.raw)

    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        fmt = '>' if is_bigendian else '<'

        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in self._DATATYPES:
                print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
            else:
                datatype_fmt, datatype_length = self._DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length

        return fmt

    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        """
        Read points from a PointCloud2 message.
        @param cloud: The point cloud to read from.
        @type  cloud: sensor_msgs.PointCloud2
        @param field_names: The names of fields to read. If None, read all fields. [default: None]
        @type  field_names: iterable
        @param skip_nans: If True, then don't return any point with a NaN value.
        @type  skip_nans: bool [default: False]
        @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
        @type  uvs: iterable
        @return: Generator which yields a list of values for each point.
        @rtype:  generator
        """
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        unpack_from = struct.Struct(fmt).unpack_from

        if skip_nans:
            if uvs:
                for u, v in uvs:
                    p = unpack_from(data, (row_step * v) + (point_step * u))
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        p = unpack_from(data, offset)
                        has_nan = False
                        for pv in p:
                            if isnan(pv):
                                has_nan = True
                                break
                        if not has_nan:
                            yield p
                        offset += point_step
        else:   
            if uvs:
                for u, v in uvs:
                    yield unpack_from(data, (row_step * v) + (point_step * u))
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        yield unpack_from(data, offset)
                        offset += point_step