/*
 * track_manager_color.h
 *
 *      Author: Alex Teichman
 *
 * I/O functionality for reading/writing a sequence of point clouds to disk.
 *
 */

#ifndef __PRECISION_TRACKING__TRACK_MANAGER_COLOR_H
#define __PRECISION_TRACKING__TRACK_MANAGER_COLOR_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <float.h>

#include <Eigen/Eigen>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace precision_tracking {

namespace track_manager_color {

const int POINTCLOUD_SERIALIZATION_VERSION = 1;
const int TRACK_SERIALIZATION_VERSION = 2;
const int TRACKMANAGER_SERIALIZATION_VERSION = 2;
const int FRAME_SERIALIZATION_VERSION = 0;

  class Frame {
  public:
    int serialization_version_;

    // Points for the object observed in a single frame, in local coordinates.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    // Time that this object was observed.
    double timestamp_;

    Frame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double timestamp);
    Frame(std::istream& istrm);
    void serialize(std::ostream& out) const;
    bool deserialize(std::istream& istrm);

    //! Returns false if there were no points.
    bool operator!=(const Frame& fr);
    bool operator==(const Frame& fr);
    Eigen::Vector3f getCentroid();
    Eigen::MatrixXf getBoundingBox();
    double getDistance();

  private:
    //! For caching of getCentroid() call.
    boost::shared_ptr<Eigen::Vector3f> centroid_;
    //! For caching of getBoundingBox() call.
    //! bounding_box_.col(0) are the small x and y coords; .col(1) are the large.
    boost::shared_ptr<Eigen::MatrixXf> bounding_box_;
  };
 
  class Track {
  public:
    int serialization_version_;
    int track_num_;
    std::string label_;
    std::vector< boost::shared_ptr<Frame> > frames_;
    
    //! Initializes with label == "unknown", and that's it.
    Track();
    Track(std::istream& istrm);
    Track(const std::string& label, const std::vector< boost::shared_ptr<Frame> >& frames);

    //! Reserves space in the vectors of velo centers, timestamps, and clouds.
    void reserve(size_t num);
    void insertFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
             double timestamp);
    bool operator==(const Track& tr);
    bool operator!=(const Track& tr);
    void serialize(std::ostream& out) const;
    bool deserialize(std::istream& istrm);
    double getMeanNumPoints() const;
    double getMeanDistance();
  };

/**
 * The TrackManagerColor class manages and manipulates tracks associated with colored point clouds.
 */
class TrackManagerColor {
public:
    int serialization_version_;                                 // Serialization version of the TrackManagerColor object
    std::vector<boost::shared_ptr<Track>> tracks_;              // Collection of tracks managed by the TrackManagerColor instance

    //! Returns the maximum number of clouds in any track.
    size_t getMaxNumClouds() const;

    //! Returns the total number of clouds across all tracks.
    size_t getNumClouds() const;

    //! Returns the number of labeled clouds among all tracks.
    size_t getNumLabeledClouds() const;

    //! Checks equality between two TrackManagerColor instances.
    bool operator==(const TrackManagerColor& tm);

    //! Checks inequality between two TrackManagerColor instances.
    bool operator!=(const TrackManagerColor& tm);

    //! Saves the TrackManagerColor object to a file specified by the filename.
    bool save(const std::string& filename);

    //! Serializes the TrackManagerColor object to an output stream.
    void serialize(std::ostream& out);

    //! Deserializes the TrackManagerColor object from an input stream.
    bool deserialize(std::istream& istrm);

    //! Deserializes a specific track from the input stream.
    bool deserialize(std::istream& istrm, const int tracknum);

    //! Sorts the tracks in descending order based on their length.
    void sortTracks();

    //! Sorts the tracks in descending order based on a custom rating function.
    void sortTracks(double (*rateTrack)(const Track&));

    //! Sorts the tracks in descending order based on pre-calculated ratings.
    void sortTracks(const std::vector<double>& track_ratings);

    //! Inserts a Track object into the TrackManagerColor instance.
    void insertTrack(boost::shared_ptr<Track> track);

    //! Reserves memory for a specified number of tracks.
    void reserve(size_t size);

    //! Default constructor for TrackManagerColor.
    TrackManagerColor();

    //! Constructs a TrackManagerColor object from a file and a track number.
    TrackManagerColor(const std::string& filename, const int tracknum);

    //! Constructs a TrackManagerColor object from a file.
    TrackManagerColor(const std::string& filename);

    //! Constructs a TrackManagerColor object from an input stream.
    TrackManagerColor(std::istream& istrm);

    //! Constructs a TrackManagerColor object from a vector of Track pointers.
    TrackManagerColor(const std::vector<boost::shared_ptr<Track>>& tracks);
};

//! Checks if the next line in the input stream matches the expected input.
bool checkLine(std::istream& istrm, const std::string& expected_input);

//! Reads a point cloud from the input stream.
bool readCloud(std::istream& s, pcl::PCLPointCloud2& cloud);

//! Deserializes a point cloud from the input stream.
void deserializePointCloud(std::istream& istrm, pcl::PointCloud<pcl::PointXYZRGB>& point_cloud);

//! Writes a point cloud to the output stream.
inline std::ostream& writeCloud(std::ostream& s, const pcl::PCLPointCloud2& cloud);

//! Serializes a point cloud to the output stream.
void serializePointCloud(const pcl::PointCloud<pcl::PointXYZRGB> cloud, std::ostream& out);

//! Checks if two point clouds are equal.
bool cloudsEqual(const pcl::PointCloud<pcl::PointXYZRGB>& c1, const pcl::PointCloud<pcl::PointXYZRGB>& c2);

//! Calculates the length of a Track.
double getTrackLength(const Track& tr);

//! Checks if two floating-point values are approximately equal.
bool floatEq(float x, float y, int maxUlps = 5);

} // namespace precision_tracking
   
#endif //__PRECISION_TRACKING__TRACK_MANAGER_COLOR_H
