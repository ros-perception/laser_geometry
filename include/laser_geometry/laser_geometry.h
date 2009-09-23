/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LASER_SCAN_UTILS_LASERSCAN_H
#define LASER_SCAN_UTILS_LASERSCAN_H

#include <map>
#include <iostream>
#include <sstream>

#include "boost/numeric/ublas/matrix.hpp"

#include "tf/tf.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud.h"

#if defined(__GNUC__)
#define DEPRECATED __attribute__((deprecated))
#else
#define DEPRECATED
#endif

namespace laser_geometry
{

  //! Enumerated output channels options.
  /*!
   * An OR'd set of these options is passed as the final argument of
   * the projectLaser and transformLaserScanToPointCloud calls to
   * enable generation of the appropriate set of additional channels.
   */
  namespace channel_option
  {
    enum ChannelOption
      {
        Intensity = 0x01, //!< Enable "intensities" channel
        Index     = 0x02, //!< Enable "index" channel
        Distance  = 0x04, //!< Enable "distances" channel
        Timestamp = 0x08, //!< Enable "stamps" channel
        Default   = (Intensity | Index) //!< Enable "intensities" and "stamps" channels
      };
  }

  DEPRECATED const int MASK_INTENSITY = channel_option::Intensity;
  DEPRECATED const int MASK_INDEX     = channel_option::Index;
  DEPRECATED const int MASK_DISTANCE  = channel_option::Distance;
  DEPRECATED const int MASK_TIMESTAMP = channel_option::Timestamp;
  DEPRECATED const int DEFAULT_MASK   = (channel_option::Intensity | channel_option::Index);
  

  //! \brief A Class to Project Laser Scan
  /*!
   * This class will project laser scans into point clouds.  It caches
   * unit vectors between runs (provided the angular resolution of
   * your scanner is not changing) to avoid excess computation.
   *
   * By default all range values less than the scanner min_range, and
   * greater than the scanner max_range are removed from the generated
   * point cloud, as these are assumed to be invalid.
   * 
   * If it is important to preserve a mapping between the index of
   * range values and points in the cloud, the recommended approach is
   * to pre-filter your laser_scan message to meet the requiremnt that all
   * ranges are between min and max_range.
   *
   * The generated PointClouds have a number of channels which can be enabled
   * through the use of ChannelOption.
   * - channel_option::Intensity - Create a channel named "intensities" with the intensity of the return for each point
   * - channel_option::Index - Create a channel named "index" containing the index from the original array for each point
   * - channel_option::Distance - Create a channel named "distances" containing the distance from the laser to each point
   * - channel_option::Timestamp - Create a channel named "stamps" containing the specific timestamp at which each point was measured
   */
  class LaserProjection
    {

    public:

      //! Destructor to deallocate stored unit vectors
      ~LaserProjection();

      

      //! Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud
      /*!
       * Project a single laser scan from a linear array into a 3D
       * point cloud.  The generated cloud will be in the same frame
       * as the original laser scan.
       *
       * \param scan_in The input laser scan
       * \param cloud_out The output point cloud
       * \param range_cutoff An additional range cutoff which can be
       *   applied which is more limiting than max_range in the scan.
       * \param channel_option An OR'd set of channels to include.
       *   Options include: channel_option::Default,
       *   channel_option::Intensity, channel_option::Index,
       *   channel_option::Distance, channel_option::Timestamp.
       */
      void projectLaser (const sensor_msgs::LaserScan& scan_in,
                         sensor_msgs::PointCloud& cloud_out,
                         double range_cutoff = -1.0,
                         int channel_options = channel_option::Default)
      {
        return projectLaser_ (scan_in, cloud_out, range_cutoff, false, channel_options);
      }

      //! Transform a sensor_msgs::LaserScan into a sensor_msgs::PointCloud in target frame
      /*!
       * Transform a single laser scan from a linear array into a 3D
       * point cloud, accounting for movement of the laser over the
       * course of the scan.  In order for this transform to be
       * meaningful at a single point in time, the target_frame must
       * be a fixed reference frame.  See the tf documentation for
       * more information on fixed frames.
       *
       * \param target_frame The frame of the resulting point cloud
       * \param scan_in The input laser scan
       * \param cloud_out The output point cloud
       * \param tf a tf::Transformer object to use to perform the
       *   transform applied which is more limiting than max_range in
       *   the scan.
       * \param channel_option An OR'd set of channels to include.
       *   Options include: channel_option::Default,
       *   channel_option::Intensity, channel_option::Index,
       *   channel_option::Distance, channel_option::Timestamp.
       */
      void transformLaserScanToPointCloud (const std::string& target_frame,
                                           const sensor_msgs::LaserScan& scan_in,
                                           sensor_msgs::PointCloud& cloud_out,
                                           tf::Transformer& tf,
                                           int channel_options = channel_option::Default)
      {
        return transformLaserScanToPointCloud_ (target_frame, cloud_out, scan_in, tf, channel_options, false);
      }

      //! Deprecated version of projectLaser
      /*!
       *  - The preservative argument has been removed.
       */
      DEPRECATED
      void projectLaser (const sensor_msgs::LaserScan& scan_in,
                         sensor_msgs::PointCloud & cloud_out,
                         double range_cutoff,
                         bool preservative,
                         int channel_options = channel_option::Default)
      {
        return projectLaser_ (scan_in, cloud_out, range_cutoff, preservative, channel_options);
      }

      //! Deprecated version of projectLaser
      /*!
       *  - The preservative argument has been removed. 
       *  - The ordering of cloud_out and scan_in have been reversed
       */
      DEPRECATED
      void transformLaserScanToPointCloud (const std::string& target_frame,
                                           sensor_msgs::PointCloud & cloud_out,
                                           const sensor_msgs::LaserScan& scan_in,
                                           tf::Transformer & tf,
                                           int channel_options = channel_option::Default,
                                           bool preservative = false)
      {
        return transformLaserScanToPointCloud_ (target_frame, cloud_out, scan_in, tf, channel_options, preservative);
      }

      //! Deprecated version of getUnitVectors
      /*!
       *  - This is now assumed to be an internal, protected API.  Do not call externally.
       */
      DEPRECATED
      const boost::numeric::ublas::matrix<double>& getUnitVectors(float angle_min,
                                                                   float angle_max,
                                                                   float angle_increment,
                                                                   unsigned int length)
      {
        return getUnitVectors_(angle_min, angle_max, angle_increment, length);
      }

    protected:

      //! Internal protected representation of getUnitVectors
      /*!
       * This function should not be used by external users, however,
       * it is left protected so that test code can evaluate it
       * appropriately.
       */
      const boost::numeric::ublas::matrix<double>& getUnitVectors_(float angle_min,
                                                                   float angle_max,
                                                                   float angle_increment,
                                                                   unsigned int length);

    private:

      //! Internal hidden representation of projectLaser
      void projectLaser_ (const sensor_msgs::LaserScan& scan_in,
                          sensor_msgs::PointCloud& cloud_out,
                          double range_cutoff,
                          bool preservative,
                          int channel_options);

      //! Internal hidden representation of transformLaserScanToPointCloud
      void transformLaserScanToPointCloud_ (const std::string& target_frame,
                                            sensor_msgs::PointCloud& cloud_out,
                                            const sensor_msgs::LaserScan& scan_in,
                                            tf::Transformer & tf,
                                            int channel_options,
                                            bool preservative);

      //! Internal map of pointers to stored values
      std::map<std::string,boost::numeric::ublas::matrix<double>* > unit_vector_map_;

    };

}

#undef DEPRECATED

#endif //LASER_SCAN_UTILS_LASERSCAN_H
