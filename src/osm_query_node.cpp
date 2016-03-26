/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      12/08/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#define OSMIUM_WITH_PBF_INPUT
#define OSMIUM_WITH_XML_INPUT

#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

#include <osmium.hpp>
#include <osmium/input/xml.hpp>
#include <osmium/handler/coordinates_for_ways.hpp>
#include <osmium/storage/objectstore.hpp>
#include <osmium/storage/byid/sparse_table.hpp>
#include <osmium/storage/byid/mmap_file.hpp>
#include <osmium/handler/coordinates_for_ways.hpp>
#include <osmium/multipolygon/assembler.hpp>
#include <osmium/geometry/multipolygon.hpp>

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <std_srvs/Empty.h>

#include <ira_open_street_map/get_distance_from_xy.h>
#include <ira_open_street_map/ecef_2_lla.h>
#include <ira_open_street_map/get_closest_way_distance_utm.h>
#include <ira_open_street_map/get_node_coordinates.h>
#include <ira_open_street_map/getDistanceFromLaneCenter.h>
#include <ira_open_street_map/getHighwayInfo.h>
#include <ira_open_street_map/latlon_2_xy.h>
#include <ira_open_street_map/lla_2_ecef.h>
#include <ira_open_street_map/snap_particle_xy.h>
#include <ira_open_street_map/way_direction.h>
#include <ira_open_street_map/xy_2_latlon.h>
#include <ira_open_street_map/oneway.h>

using namespace std;

typedef Osmium::Storage::ById::SparseTable<Osmium::OSM::Position> storage_sparsetable_t;
typedef Osmium::Storage::ById::MmapFile<Osmium::OSM::Position> storage_mmap_t;
typedef Osmium::Handler::CoordinatesForWays<storage_sparsetable_t, storage_mmap_t> cfw_handler_t;

/**
 * @brief The ObjectHandler class
 * With this class we handle the Osmium Library
 */
class ObjectHandler : public Osmium::Handler::Base
{

public:

    typedef std::set<shared_ptr<Osmium::OSM::Node const>     > nodeset;
    typedef std::set<shared_ptr<Osmium::OSM::Way const>      > wayset;
    typedef std::set<shared_ptr<Osmium::OSM::Relation const> > relationset;

    storage_sparsetable_t store_pos;
    storage_mmap_t store_neg;
    cfw_handler_t* handler_cfw;

    nodeset     m_nodes;
    wayset      m_ways;
    relationset m_relations;




    ObjectHandler() : m_nodes(), m_ways(), m_relations()
    {
        handler_cfw = new cfw_handler_t(store_pos, store_neg);
    }

    void init(Osmium::OSM::Meta& meta)
    {
        handler_cfw->init(meta);
    }

    /**
     * Insert shared_ptr of Node into object store.
     */
    void node(const shared_ptr<Osmium::OSM::Node const>& node)
    {
        handler_cfw->node(node);
        m_nodes.insert(node);
    }

    void after_nodes()
    {
        handler_cfw->after_nodes();
    }

    /**
     * Insert shared_ptr of Way into object store.
     */
    void way(const shared_ptr<Osmium::OSM::Way>& way)
    {
        handler_cfw->way(way);
        m_ways.insert(way);
    }

    /**
     * Insert shared_ptr of Relation into object store.
     */
    void relation(const shared_ptr<Osmium::OSM::Relation const>& relation)
    {
        m_relations.insert(relation);
    }

    /**
     * Remove all nodes from object store.
     */
    void clear_nodes()
    {
        m_nodes.clear();
    }

    /**
     * Remove all ways from object store.
     */
    void clear_ways()
    {
        m_ways.clear();
    }

    /**
     * Remove all relations from object store.
     */
    void clear_relations()
    {
        m_relations.clear();
    }

    /**
     * Remove all objects from object store.
     */
    void clear()
    {
        clear_nodes();
        clear_ways();
        clear_relations();
    }
};

/* Structs ---------------------------------------------------------------------------*/
/// Combination of ObjectStore and CoordinateForWays handlers
ObjectHandler oh;

/// Coordinates struct
struct Coordinates
{
    float latitude;
    float longitude;
    float altitude;
};

/// Cartesian coordinates struct
struct Xy
{
    double x;
    double y;
};

/// Response struct for Way direction calculator
struct Way_dir_struct
{
    // Direction quaternion
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    // Direction yaw
    float yaw_deg;
    float yaw_rad;

    // Create two particles with opposite direction
    bool opposite_direction;

    // Error flags
    bool way_not_found;
    bool not_a_way;
};

/* Functions ---------------------------------------------------------------------------*/

bool                    isLeft                       (Xy osmA, Xy osmB, Xy queryPoint);
bool                    ecef_2_lla                   (ira_open_street_map::ecef_2_lla::Request& req, ira_open_street_map::ecef_2_lla::Response& resp);
Coordinates             ecef2lla_helper              (float x, float y, float z);
bool                    get_closest_way_distance_utm (ira_open_street_map::get_closest_way_distance_utm::Request& req, ira_open_street_map::get_closest_way_distance_utm::Response& resp);
bool                    get_distance_from_xy         (ira_open_street_map::get_distance_from_xy::Request& req, ira_open_street_map::get_distance_from_xy::Response& resp);
double inline           get_distance_helper          (double x1, double y1, double x2, double y2);
bool                    get_node_coordinates         (ira_open_street_map::get_node_coordinates::Request& req, ira_open_street_map::get_node_coordinates::Response& resp);
ROS_DEPRECATED bool     getDistanceFromLaneCenter    (ira_open_street_map::getDistanceFromLaneCenter::Request& req, ira_open_street_map::getDistanceFromLaneCenter::Response& resp);
bool                    getHighwayInfo               (ira_open_street_map::getHighwayInfo::Request& req, ira_open_street_map::getHighwayInfo::Response& resp);
bool                    getOneWayInfo                (ira_open_street_map::oneway::Request& req, ira_open_street_map::oneway::Response& resp);
bool                    latlon_2_xy                  (ira_open_street_map::latlon_2_xy::Request& req, ira_open_street_map::latlon_2_xy::Response& resp);
Xy                      latlon2xy_helper             (double lat, double lngd);
bool                    lla_2_ecef                   (ira_open_street_map::lla_2_ecef::Request& req, ira_open_street_map::lla_2_ecef::Response& resp);
double                  lla_distance                 (Coordinates& c1, Coordinates& c2);
geometry_msgs::Point    lla2ecef_helper              (double lat, double lon, double alt);
void                    load_waylist                 ();
void                    publish_buildinglist         (ros::Publisher &markerArrayPublisher_buildings);
Xy                      snap_particle_helper         (Xy& A, Xy& B, Xy& C);
bool                    snap_particle_xy             (ira_open_street_map::snap_particle_xy::Request& req, ira_open_street_map::snap_particle_xy::Response& resp);
Way_dir_struct          way_direction_helper         (const boost::shared_ptr<Osmium::OSM::Way const>& way, Xy& A, Xy& B);
Way_dir_struct          way_direction_helper         (double way_id);
bool                    way_direction                (ira_open_street_map::way_direction::Request& req, ira_open_street_map::way_direction::Response& resp);
bool                    xy_2_latlon                  (ira_open_street_map::xy_2_latlon::Request& req, ira_open_street_map::xy_2_latlon::Response& resp);
Coordinates             xy2latlon_helper             (double x, double y, double utmz, bool southern);
double                  xyz_distance                 (geometry_msgs::Point& p1, geometry_msgs::Point& p2);


// How to get UTM from Longitude:
// UTM = 1.0 + floor((lngd + 180.0) / 6.0);
//
// (italy -> UTM = 32, southern = false)
Coordinates xy2latlon_helper(double x, double y, double utmz, bool southern)
{

    // WGS 84 datum
    double eqRad = 6378137.0;
    double flat = 298.2572236;

    // constants used in calculations:
    double a = eqRad;           // equatorial radius in meters
    double f = 1.0 / flat;        // polar flattening
    double b = a * (1.0 - f);     // polar radius
    double e = sqrt(1.0 - (pow(b, 2) / pow(a, 2))); // eccentricity
    double k0 = 0.9996;
    double k = 1;
    double drad = M_PI / 180.0;

    double esq = (1.0 - (b / a) * (b / a));
    double e0sq = e * e / (1.0 - e * e);
    double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;                         // Central meridian of zone
    double e1 = (1 - sqrt(1 - e * e) ) / (1.0 + sqrt(1 - e * e));
    double M0 = 0.0;
    double M = 0.0;

    if (!southern)
        M = M0 + y / k0;    // Arc length along standard meridian.
    else
        M = M0 + (y - 10000000.0) / k;

    double mu = M / (a * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0))));
    double phi1 = mu + e1 * (3.0 / 2.0 - 27.0 * e1 * e1 / 32.0) * sin(2.0 * mu) + e1 * e1 * (21.0 / 16.0 - 55.0 * e1 * e1 / 32.0) * sin(4.0 * mu);   //Footprint Latitude
    phi1 = phi1 + e1 * e1 * e1 * (sin(6.0 * mu) * 151.0 / 96.0 + e1 * sin(8.0 * mu) * 1097.0 / 512.0);

    double C1 = e0sq * cos(phi1) * cos(phi1);
    double T1 = tan(phi1) * tan(phi1);
    double N1 = a / sqrt(1.0f - pow(e * sin(phi1), 2.0f));
    double R1 = N1 * (1.0f - pow(e, 2.0f)) / (1.0f - pow(e * sin(phi1), 2.0f));
    double D = (x - 500000.0) / (N1 * k0);
    double phi = (D * D) * (1.0 / 2.0 - D * D * (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * e0sq) / 24.0);
    phi = phi + pow(D, 6.0) * (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * e0sq - 3.0 * C1 * C1) / 720.0;
    phi = phi1 - (N1 * tan(phi1) / R1) * phi;

    double lat = floor(1000000.0 * phi / drad) / 1000000.0;
    double lng = D * (1.0 + D * D * ((-1.0 - 2.0 * T1 - C1) / 6.0 + D * D * (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * e0sq + 24.0 * T1 * T1) / 120.0)) / cos(phi1);
    double lngd = zcm + lng / drad;

    Coordinates coords = { lat, lngd };
    return coords;
}

// Conversion between geographic and UTM coordinates
// Adapted from:
// http://www.uwgb.edu/dutchs/UsefulData/ConvertUTMNoOZ.HTM
Xy latlon2xy_helper(double lat, double lngd)
{

    // WGS 84 datum
    double eqRad = 6378137.0;
    double flat = 298.2572236;

    // constants used in calculations:
    double a = eqRad;           // equatorial radius in meters
    double f = 1.0 / flat;        // polar flattening
    double b = a * (1.0 - f);     // polar radius
    double e = sqrt(1.0 - (pow(b, 2) / pow(a, 2))); // eccentricity
    double k0 = 0.9996;
    double drad = M_PI / 180.0;

    double phi = lat * drad;   // convert latitude to radians
    double utmz = 1.0 + floor((lngd + 180.0) / 6.0); // longitude to utm zone
    double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;     // central meridian of a zone
    double esq = (1.0 - (b / a) * (b / a));
    double e0sq = e * e / (1.0 - e * e);
    double M = 0.0;
    double M0 = 0.0;
    double N = a / sqrt(1.0 - pow(e * sin(phi), 2));
    double T = pow(tan(phi), 2);
    double C = e0sq * pow(cos(phi), 2);
    double A = (lngd - zcm) * drad * cos(phi);

    // calculate M (USGS style)
    M = phi * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0)));
    M = M - sin(2.0 * phi) * (esq * (3.0 / 8.0 + esq * (3.0 / 32.0 + 45.0 * esq / 1024.0)));
    M = M + sin(4.0 * phi) * (esq * esq * (15.0 / 256.0 + esq * 45.0 / 1024.0));
    M = M - sin(6.0 * phi) * (esq * esq * esq * (35.0 / 3072.0));
    M = M * a; // Arc length along standard meridian

    // now we are ready to calculate the UTM values...
    // first the easting (relative to CM)
    double x = k0 * N * A * (1.0 + A * A * ((1.0 - T + C) / 6.0 + A * A * (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e0sq) / 120.0));
    x = x + 500000.0; // standard easting

    // now the northing (from the equator)
    double y = k0 * (M - M0 + N * tan(phi) * (A * A * (1.0 / 2.0 + A * A * ((5.0 - T + 9.0 * C + 4.0 * C * C) / 24.0 + A * A * (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e0sq) / 720.0))));
    if (y < 0)
    {
        y = 10000000.0 + y; // add in false northing if south of the equator
    }
    //double easting  = round(10.0 * x) / 10.0;
    //double northing = round(10.0 * y) / 10.0;

    double easting  = x;
    double northing = y;

    Xy coords;
    coords.x = easting;
    coords.y = northing;

    return coords;

}

/**
 * LLA2ECEF - convert latitude, longitude, and altitude to
 *             earth-centered, earth-fixed (ECEF) cartesian
 *
 * USAGE:
 * [x,y,z] = lla2ecef(lat,lon,alt)
 *
 * x = ECEF X-coordinate (m)
 * y = ECEF Y-coordinate (m)
 * z = ECEF Z-coordinate (m)
 * lat = geodetic latitude (radians)
 * lon = longitude (radians)
 * alt = height above WGS84 ellipsoid (m)
 *
 * Notes: This function assumes the WGS84 model.
 *        Latitude is customary geodetic (not geocentric).
 *
 * Source: "Department of Defense World Geodetic System 1984"
 *         Page 4-4
 *         National Imagery and Mapping Agency
 *         Last updated June, 2004
 *         NIMA TR8350.2
 *
 * Michael Kleder, July 2005
 * (C++ porting by Dario Limongi)
 */
geometry_msgs::Point lla2ecef_helper(double lat, double lon, double alt)
{
    geometry_msgs::Point point;

    // WGS84 ellipsoid constants:
    double a = 6378137;            // (m) at the equator
    double e = 0.081819190842622;  // eccentricity

    // intermediate calculation
    // (prime vertical radius of curvature)
    double N = a / sqrt(1 - (e * e) * (sin(lat) * sin(lat)));

    // results:
    point.x = (N + alt) * cos(lat) * cos(lon);
    point.y = (N + alt) * cos(lat) * sin(lon);
    point.z = ((1 - (e * e)) * N + alt) * sin(lat);

    return point;
}

/**
 * ECEF2LLA - convert earth-centered earth-fixed (ECEF)
 *            cartesian coordinates to latitude, longitude,
 *            and altitude
 *
 * USAGE:
 * [lat,lon,alt] = ecef2lla(x,y,z)
 *
 * lat = geodetic latitude (radians)
 * lon = longitude (radians)
 * alt = height above WGS84 ellipsoid (m)
 * x = ECEF X-coordinate (m)
 * y = ECEF Y-coordinate (m)
 * z = ECEF Z-coordinate (m)
 *
 * Notes: (1) This function assumes the WGS84 model.
 *        (2) Latitude is customary geodetic (not geocentric).
 *        (3) Inputs may be scalars, vectors, or matrices of the same
 *            size and shape. Outputs will have that same size and shape.
 *        (4) Tested but no warranty; use at your own risk.
 *        (5) Michael Kleder, April 2006
 * (C++ porting by Dario Limongi)
 */
Coordinates ecef2lla_helper(float x, float y, float z)
{
    Coordinates coords;

    // WGS84 ellipsoid constants:
    float a = 6378137;            // (m) at the equator
    float e = 0.081819190842622;  // eccentricity

    // calculations:
    float b   = sqrt( (a * a) * (1 - e * e ));
    float ep  = sqrt( (a * a - b * b ) / (b * b));
    float p   = sqrt( x * x + y * y );
    float th  = atan2(a * z, b * p);
    float lon = atan2(y, x);
    float lat = atan2( (z + ep * ep * b * sin(th) * sin(th) * sin(th)), (p - e * e * a * cos(th) * cos(th) * cos(th)) );
    float N   = a / sqrt( 1 - e * e * sin(lat) * sin(lat));
    float alt = p / cos(lat) - N;

    // return lon in range [0,2*pi)
    lon = lon - (M_PI * 2) * floor( lon / (M_PI * 2) );

    coords.latitude = lat;
    coords.longitude = lon;
    coords.altitude = alt;

    return coords;
}

/**
 * @brief get_distance_helper
 * @param x1 1st point, x coordinate
 * @param y1 1st point, y coordinate
 * @param x2 2nd point, x coordinate
 * @param y2 2nd point, y coordinate
 * @return value of the distance between the two points
 */
double inline get_distance_helper(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(fabs(dx * dx + dy * dy));
}

/**
 * @brief snapxy2way_outbounds_helper
 * @param A 1st node of the way
 * @param B 2nd node of the way
 * @param C hypothesis position
 * @return the position of the orthogonal projection of C in the segment defined by A and B
 *
 * Snaps the particle position to the way segment defined by A and B using
 * the orthogonal projection
 */
Xy snap_particle_helper(Xy& A, Xy& B, Xy& C)
{

    // Snapped point to segment A,B
    Xy D;

    // vector from A to B
    Xy AB = {B.x - A.x, B.y - A.y };

    // squared distance from A to B
    double AB_squared = AB.x * AB.x + AB.y * AB.y;

    if (AB_squared == 0)
    {
        // A and B are the same point
        D.x = A.x;
        D.y = A.y;
    }
    else
    {
        // vector from A to p
        Xy Ap = {C.x - A.x, C.y - A.y};

        // Consider the line extending the segment, parameterized as A + t (B - A)
        // We find projection of point p onto the line.
        // It falls where t = [(p-A) . (B-A)] / |B-A|^2
        double t = (Ap.x * AB.x + Ap.y * AB.y) / AB_squared;

        if (t < 0.0)
        {
            // "Before" A on the line, just return A
            // q = A;
            D.x = A.x;
            D.y = A.y;
        }
        else if (t > 1.0)
        {
            // "After" B on the line, just return B
            // q = B;
            D.x = B.x;
            D.y = B.y;
        }
        else
        {
            // projection lines "inbetween" A and B on the line
            D.x = t * AB.x + A.x;
            D.y = t * AB.y + A.y;
        }
    }

    return D;
}

/**
 * @brief way_direction_helper
 * @param way this is closest way to the current hypothesis
 * @param A min_dist_node1, following the order of the elements in the way, this is the 1st element
 * @param B min_dist_node2, following the order of the elements in the way, this is the 2nd element
 * @return
 *
 * Given a way (and two XY points) return, as a service response, the direction of the hypotheses,
 * based on the two points that creates the segment (A and B points). It also
 * considers if the way is oneway or not.
 *
 */
Way_dir_struct way_direction_helper(const boost::shared_ptr<Osmium::OSM::Way const>& way, Xy& A, Xy& B)
{
    // Init response
    Way_dir_struct response;
    response.not_a_way = false;
    response.way_not_found = false;
    response.q_w = 0;
    response.q_x = 0;
    response.q_y = 0;
    response.q_z = 0;
    response.yaw_deg = 0;
    response.yaw_rad = 0;
    response.opposite_direction = 0;

    double theta = 0;                // Angle between A an B;
    bool opposite_particles = false; // This tells if we should create 2 particles with opposite directions

    // Find tags
    //    const char* lanes = way->tags().get_value_by_key("lanes");
    const char* one_way = way->tags().get_value_by_key("oneway");

    // Get angle between A and B (radians)
    theta = atan2(B.y - A.y, B.x - A.x);

    // ONEWAY TAG FOUND:
    if ( one_way )
    {

        if (strcmp(one_way, "yes") == 0 || strcmp(one_way, "true") == 0 || strcmp(one_way, "1") == 0)
        {
            opposite_particles = false;
        }
        else if (strcmp(one_way, "no") == 0 || strcmp(one_way, "false") == 0 || strcmp(one_way, "0") == 0)
        {
            // If street has multiple lanes, then we should create 2 particles with opposite direction
            opposite_particles = true;
        }
        else if (strcmp(one_way, "-1") == 0 || strcmp(one_way, "reverse") == 0)
        {
            // Get angle between B and A (radians)
            theta = atan2(A.y - B.y, A.x - B.x);
            opposite_particles = false;
        }
    }
    // NO TAGS FOUND!
    else
    {
        // Create 2 particles, since we don't know anything about this way
        opposite_particles = true;
    }


    // Create quaternion from angle and put it inside response
    //        Normalize angle if it is negative:
    if (theta < 0)
    {
        theta += 2 * M_PI;
    }
    tf::Quaternion q = tf::createQuaternionFromYaw(theta);

    // Set response quaternion value
    response.q_w = q.getW();
    response.q_x = q.getX();
    response.q_y = q.getY();
    response.q_z = q.getZ();

    // Set response yaw value
    response.yaw_deg = theta * 180.0f / M_PI;
    response.yaw_rad = theta;

    // Set opposite_direction flag
    response.opposite_direction = opposite_particles;

    ROS_DEBUG_STREAM("   Direction calculated, Way ID: " << boost::lexical_cast<std::string>(way->id()));
    ROS_DEBUG_STREAM("   Theta: " << theta);
    ROS_DEBUG_STREAM("   Degrees: " << response.yaw_deg);
    ROS_DEBUG_STREAM("   Quaternion: " << q.getX() << " " << q.getY() << " " << q.getZ() << " " << q.getW() << "\n");

    return response;
}

/**
 * @brief way_direction_helper
 * @param way_id
 * @return
 *
 * Given a way id, find that way and create a RESPONSE with it,
 * calculating the orientation using the FIRST two nodes.
 */
Way_dir_struct way_direction_helper(double way_id)
{
    // Init response
    Way_dir_struct response;
    response.not_a_way = false;
    response.way_not_found = false;
    response.q_w = 0;
    response.q_x = 0;
    response.q_y = 0;
    response.q_z = 0;
    response.yaw_deg = 0;
    response.yaw_rad = 0;

    double theta = 0;                // Angle between A an B;
    bool opposite_particles = false; // This tells if we should create 2 particles with opposite directions
    bool found = false;              // This flag tells if we found they Way inside our map or not


    // Cycle through every stored way
    for (std::set<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = oh.m_ways.begin(); way_itr != oh.m_ways.end(); way_itr++)
    {
        if ( (*way_itr)->id() == way_id  )
        {
            ROS_DEBUG_STREAM("SERVICE way_direction:");

            // way id found, check if it's a street
            const char* highway = (*way_itr)->tags().get_value_by_key("highway");

            if (highway)
            {
                // Get way node list
                Osmium::OSM::WayNodeList waylist = (*way_itr)->nodes();

                //                const char* lanes = (*way_itr)->tags().get_value_by_key("lanes");
                const char* one_way = (*way_itr)->tags().get_value_by_key("oneway");

                // Get first and second node UTM coords
                Xy A = latlon2xy_helper(waylist.begin()->position().lat(), waylist.begin()->position().lon());
                Xy B = latlon2xy_helper((++waylist.begin())->position().lat(), (++waylist.begin())->position().lon());
                theta = atan2(B.y - A.y, B.x - A.x);

                // ONEWAY TAG FOUND:
                if ( one_way )
                {
                    if (strcmp(one_way, "yes") == 0 || strcmp(one_way, "true") == 0 || strcmp(one_way, "1") == 0)
                    {
                        opposite_particles = false;
                    }
                    else if (strcmp(one_way, "no") == 0 || strcmp(one_way, "false") == 0 || strcmp(one_way, "0") == 0)
                    {
                        // If street has multiple lanes, then we should create 2 particles with opposite direction
                        opposite_particles = true;
                    }
                    else if (strcmp(one_way, "-1") == 0 || strcmp(one_way, "reverse") == 0)
                    {
                        // Get angle between B and A (radians)
                        theta = atan2(A.y - B.y, A.x - B.x);
                        opposite_particles = false;
                    }
                }
                // NO TAGS FOUND!
                else
                {
                    // Create 2 particles, since we don't know anything about this way
                    opposite_particles = true;
                }
            }
            else
            {
                ROS_DEBUG_STREAM("   Given ID is not a Way");
                response.not_a_way = true;
                return response;
            }

            // Way was found, exit FOR loop
            found = true;
            break;
        }
    }

    if (found)
    {
        // Create quaternion from angle and put it inside response
        //        Normalize angle if it is negative:
        if (theta < 0)
        {
            theta += 2 * M_PI;
        }
        tf::Quaternion q = tf::createQuaternionFromYaw(theta);

        // Set response quaternion value
        response.q_w = q.getW();
        response.q_x = q.getX();
        response.q_y = q.getY();
        response.q_z = q.getZ();

        // Set response yaw value
        response.yaw_deg = theta * 180.0f / M_PI;
        response.yaw_rad = theta;

        // Set opposite_direction flag
        response.opposite_direction = opposite_particles;

        //        ROS_DEBUG_STREAM("   Direction calculated, Way ID: " << boost::lexical_cast<std::string>(way_id));
        //        ROS_DEBUG_STREAM("   Theta: " << theta);
        //        ROS_DEBUG_STREAM("   Degrees: " << response.yaw_deg);
        //        ROS_DEBUG_STREAM("   Quaternion: " << q.getX() << " " << q.getY() << " " << q.getZ()<< " " << q.getW());
        //        ROS_DEBUG_STREAM("   Opposite_particles: " << opposite_particles);

        return response;
    }
    else
    {
        // Given ID was not found inside our Map
        ROS_DEBUG_STREAM("   Given ID was not found inside our Map");
        response.way_not_found = true;
        return response;
    }
}

/**
 * @brief oneWay returns true if the way_id has the tag OneWay, false otherwise
 * @param way_id, inside the request(req.way_id)
 * @return boolean: true if oneway tag found, false otherwise
 *
 * Returns true if oneway tag found. If returns true, check the response;
 *
 */
bool getOneWayInfo(ira_open_street_map::oneway::Request& req, ira_open_street_map::oneway::Response& resp)
{
    resp.oneway = false; //initialize response to false, default value if wayid is not found

    // Cycle through every stored way
    for (std::set<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = oh.m_ways.begin(); way_itr != oh.m_ways.end(); way_itr++)
    {
        if ( (*way_itr)->id() == req.way_id  )
        {
            ROS_DEBUG_STREAM("SERVICE oneWay");

            // way id found, check if it's a street
            const char* highway = (*way_itr)->tags().get_value_by_key("highway");

            if (highway)
            {
                const char* one_way = (*way_itr)->tags().get_value_by_key("oneway");

                // ONEWAY TAG FOUND:
                if ( one_way )
                {
                    if (strcmp(one_way, "yes") == 0 || strcmp(one_way, "true") == 0 || strcmp(one_way, "1") == 0)
                    {
                        resp.oneway = true;
                    }
                    else if (strcmp(one_way, "no") == 0 || strcmp(one_way, "false") == 0 || strcmp(one_way, "0") == 0)
                    {
                        resp.oneway = false;
                    }
                    else if (strcmp(one_way, "-1") == 0 || strcmp(one_way, "reverse") == 0)
                    {
                        resp.oneway = true;
                    }
                    return true;
                }
                // NO ONEWAY TAG FOUND!
                else
                {
                    // in this case, the oneway tag is not present. return false
                    resp.oneway = false;
                    return false;
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Given ID is not a Way!");
                resp.oneway = false;
                return false;
            }
        }
    }

    return false;
}

double xyz_distance(geometry_msgs::Point& p1, geometry_msgs::Point& p2)
{
    // calculate difference between points
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;

    // If the points are so close that the curvature of the Earth doesn't
    // matter, then we can simply calculate the stright-line distance:
    return sqrt(dx * dx + dy * dy + dz * dz);
}

double lla_distance(Coordinates& c1, Coordinates& c2)
{
    // convert LLA to ECEF
    geometry_msgs::Point p1 = lla2ecef_helper(c1.latitude, c1.longitude, c1.altitude);
    geometry_msgs::Point p2 = lla2ecef_helper(c2.latitude, c2.longitude, c2.altitude);

    // calculate difference between points
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;

    // If the points are so close that the curvature of the Earth doesn't
    // matter, then we can simply calculate the stright-line distance:
    return sqrt(dx * dx + dy * dy + dz * dz);
}





/**
 * @brief snap_particle_xy  XY in global map frame! (Northing/Easting)
 * @param req
 * @param resp
 * @return
 */
bool snap_particle_xy(ira_open_street_map::snap_particle_xy::Request& req, ira_open_street_map::snap_particle_xy::Response& resp)
{

    // (1) Calculate distance between particle and every map Way (keep only ways that are below 'max_distance_radius')
    vector<shared_ptr<Osmium::OSM::Way const> > way_vector; /// keep OSM way stored
    for (std::set<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = oh.m_ways.begin(); way_itr != oh.m_ways.end(); way_itr++)
    {
        // Use only ways with Key:Highway
        const char* highway = (*way_itr)->tags().get_value_by_key("highway");
        if (!highway)
            continue;

        // Get way-node list
        Osmium::OSM::WayNodeList waylist = (*way_itr)->nodes();
        for (Osmium::OSM::WayNodeList::iterator node_list_itr = waylist.begin(); node_list_itr != waylist.end(); node_list_itr++ )
        {
            // Get way-node coordinates
            double lat = node_list_itr->position().lat();
            double lon = node_list_itr->position().lon();
            Xy way_node_xy = latlon2xy_helper(lat, lon);

            // Calculate distance from particle
            double distance = get_distance_helper(req.x, req.y, way_node_xy.x, way_node_xy.y);

            // Check if it's below max_distance_radius
            if (distance <= req.max_distance_radius)
            {
                // Store way ID+
                way_vector.push_back(*way_itr);
                break;
            }
        }
    }

    if (way_vector.size() == 0)
    {
        ROS_ERROR_STREAM("     No map nodes found next to particle. Distance radius: " << req.max_distance_radius << " m");
        return false;
    }

    //    //      Print kept Ways ID
    //    ROS_DEBUG_STREAM("PRINT Way ID vector");
    //    for(vector<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = way_vector.begin(); way_itr != way_vector.end(); way_itr++){
    //        ROS_DEBUG_STREAM("   Way ID: " << boost::lexical_cast<std::string>((*way_itr)->id()) );
    //    }

    // (2) For every kept way, calculate shortest distance between particle and every way's segments (a way segment is defined by 2 nodes)
    //        init values
    double min_distance = 999999999;
    shared_ptr<Osmium::OSM::Way const> min_distance_way;
    Xy snapped_xy;
    Xy particle = {req.x, req.y};
    Xy min_dist_node1;
    Xy min_dist_node2;
    int64 way_id = 0;

    //        start cycle
    for (vector<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = way_vector.begin(); way_itr != way_vector.end(); way_itr++)
    {
        // Use only ways with Key:Highway
        // WARNING: this should be useless since we added only ways with Key:highway in the phase
        const char* highway = (*way_itr)->tags().get_value_by_key("highway");
        if (!highway)
            continue;

        Osmium::OSM::WayNodeList way_n_list = (*way_itr)->nodes();
        for (Osmium::OSM::WayNodeList::iterator node_list_itr = way_n_list.begin(); node_list_itr != --way_n_list.end(); node_list_itr++ )
        {
            // extract 1st node coordinates
            double lat = node_list_itr->position().lat();
            double lon = node_list_itr->position().lon();
            Xy node_1 = latlon2xy_helper(lat, lon);

            // extract 2nd node coordinates
            Osmium::OSM::WayNodeList::iterator it2 = node_list_itr;
            it2++;
            double lat2 = it2->position().lat();
            double lon2 = it2->position().lon();
            Xy node_2 = latlon2xy_helper(lat2, lon2);

            // snap particle (on the way=highway segment defined by node_1 and node_2)
            Xy snap = snap_particle_helper(node_1, node_2, particle);

            // calculate distance from particle to snapped way segment
            double dist = get_distance_helper(snap.x, snap.y, particle.x, particle.y);

            if (dist < min_distance)
            {
                // update min distance
                min_distance = dist;
                min_distance_way = *way_itr;
                snapped_xy = snap;
                min_dist_node1 = node_1;
                min_dist_node2 = node_2;
                way_id = (*way_itr)->id();
            }

            // break if it's last way segment
            if ( it2 == way_n_list.end() )
            {
                break;
            }
        }
    }

    // Get snapped particle direction using the min_distance segment of the nearest way=highway element
    Way_dir_struct dir = way_direction_helper(min_distance_way, min_dist_node1, min_dist_node2);

    // Set response values
    resp.snapped_x = snapped_xy.x;
    resp.snapped_y = snapped_xy.y;
    resp.way_dir_rad = dir.yaw_rad;
    resp.way_dir_degrees = dir.yaw_deg;
    resp.way_dir_quat_w = dir.q_w;
    resp.way_dir_quat_x = dir.q_x;
    resp.way_dir_quat_y = dir.q_y;
    resp.way_dir_quat_z = dir.q_z;
    resp.way_dir_opposite_particles = dir.opposite_direction;
    resp.distance_from_way = min_distance;
    resp.way_id = way_id;
    resp.isLeft = isLeft(min_dist_node1, min_dist_node2, particle); // #538 checks if we're on the left of right w.r.t the WAY

    // #538 std::cout<< "LEFT/RIGHT: " << isLeft(min_dist_node1, min_dist_node2, particle) << std::endl;

    return true;
}


/**
 * @brief isLeft This routine checks on which side of a segment created by two points
 * a queryPoint is. This is related to task #538
 * @param osmA the first  point of the orientated segment, usually northing/easting
 * @param osmB the second point of the orientated segment, usually northing/easting
 * @param queryPoint the query point, position of the particle, usually northing/easting
 * @return true if the queryPoint is on the LEFT, false if it is on the RIGHT.
 */
inline bool isLeft(Xy osmA, Xy osmB, Xy queryPoint)
{
    return ((osmB.x - osmA.x) * (queryPoint.y - osmA.y) - (osmB.y - osmA.y) * (queryPoint.x - osmA.x)) > 0;
}


/**
 * @brief way_direction
 * 1. Is there Oneway tag?
 *   YES:
 *      oneway = yes : calculate way direction with atan2 using Way node list sorting
 *      oneway = no :  calculate way direction with atan2 using Way node list sorting and place 2 particles with opposite directions
 *      oneway = -1 :  calculate way direction with atan2 using Way node list sorting and reverse the direction
 *   NO: Go to (2)
 *
 * 2. Is there Lanes tag?
 *   YES:
 *      lanes = 1 : calculate way direction with atan2 using Way node list sorting
 *      lanes > 1 : calculate way direction with atan2 using Way node list sorting and place 2 particles with opposite directions
 *   NO: Go to (3)
 *
 * 3. calculate way direction with atan2 using Way node list sorting and place 2 particles with opposite directions
 *
 * @param req
 * @param resp
 * @return
 */
bool way_direction(ira_open_street_map::way_direction::Request& req, ira_open_street_map::way_direction::Response& resp)
{

    ROS_DEBUG_STREAM("SERVICE way_direction:");
    Way_dir_struct response = way_direction_helper(req.way_id);

    // check errors
    if (response.way_not_found || response.not_a_way)
    {
        return false;
    }
    else
    {
        // Set quaternion
        resp.quat_w = response.q_w;
        resp.quat_x = response.q_x;
        resp.quat_y = response.q_y;
        resp.quat_z = response.q_z;

        // Set Yaw
        resp.yaw_deg = response.yaw_deg;
        resp.yaw_rad = response.yaw_rad;

        // Set opposite particles flag
        resp.opposite_direction = response.opposite_direction;

        return true;
    }
}


/**
 * @brief get_closest_way_distance_utm
 * @param req
 * @param resp
 * @return
 */
bool get_closest_way_distance_utm(ira_open_street_map::get_closest_way_distance_utm::Request& req, ira_open_street_map::get_closest_way_distance_utm::Response& resp)
{

    // (1) Calculate distance between particle and every map Way (keep only ways that are below 'max_distance_radius')
    vector<shared_ptr<Osmium::OSM::Way const> > way_vector; /// keep OSM way stored
    for (std::set<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = oh.m_ways.begin(); way_itr != oh.m_ways.end(); way_itr++)
    {
        // Use only ways with Key:Highway
        const char* highway = (*way_itr)->tags().get_value_by_key("highway");
        if (!highway)
            continue;

        // Get way-node list
        Osmium::OSM::WayNodeList waylist = (*way_itr)->nodes();
        for (Osmium::OSM::WayNodeList::iterator node_list_itr = waylist.begin(); node_list_itr != waylist.end(); node_list_itr++ )
        {
            // Get way-node coordinates
            double lat = node_list_itr->position().lat();
            double lon = node_list_itr->position().lon();
            Xy way_node_xy = latlon2xy_helper(lat, lon);

            // Calculate distance from particle
            double distance = get_distance_helper(req.x, req.y, way_node_xy.x, way_node_xy.y);

            // Check if it's below max_distance_radius
            if (distance <= req.max_distance_radius)
            {
                // Store way ID+
                way_vector.push_back(*way_itr);
                break;
            }
        }
    }

    if (way_vector.size() == 0)
    {
        ROS_ERROR_STREAM("     No map nodes found next to particle. Distance radius: " << req.max_distance_radius << " m");
        return false;
    }

    // (2) For every kept way, calculate shortest distance between particle and every way's segments (a way segment is defined by 2 nodes)
    //        init values
    double min_distance = 999999999;
    shared_ptr<Osmium::OSM::Way const> min_distance_way;
    Xy min_distance_xy;
    Xy particle = {req.x, req.y};
    Xy min_dist_node1;
    Xy min_dist_node2;

    //        start cycle
    for (vector<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = way_vector.begin(); way_itr != way_vector.end(); way_itr++)
    {
        // Use only ways with Key:Highway
        // WARNING: this should be useless since we added only ways with Key:highway in the phase
        const char* highway = (*way_itr)->tags().get_value_by_key("highway");
        if (!highway)
            continue;

        Osmium::OSM::WayNodeList way_n_list = (*way_itr)->nodes();
        for (Osmium::OSM::WayNodeList::iterator node_list_itr = way_n_list.begin(); node_list_itr != --way_n_list.end(); node_list_itr++ )
        {
            // extract 1st node coordinates
            double lat = node_list_itr->position().lat();
            double lon = node_list_itr->position().lon();
            Xy node_1 = latlon2xy_helper(lat, lon);

            // extract 2nd node coordinates
            Osmium::OSM::WayNodeList::iterator it2 = node_list_itr;
            it2++;
            double lat2 = it2->position().lat();
            double lon2 = it2->position().lon();
            Xy node_2 = latlon2xy_helper(lat2, lon2);

            // snap particle
            Xy snap = snap_particle_helper(node_1, node_2, particle);

            // calculate distance from particle to way segment
            double dist = get_distance_helper(snap.x, snap.y, particle.x, particle.y);

            if (dist < min_distance)
            {
                // update min distance
                min_distance = dist;
                min_distance_way = *way_itr;
                min_distance_xy = snap;
                min_dist_node1 = node_1;
                min_dist_node2 = node_2;
            }

            // break if it's last way segment
            if ( it2 == way_n_list.end() )
            {
                break;
            }
        }
    }

    // Set response value
    resp.distance = min_distance;

    return true;
}


bool latlon_2_xy(ira_open_street_map::latlon_2_xy::Request& req, ira_open_street_map::latlon_2_xy::Response& resp)
{

    if ((req.latitude > 90) || (req.latitude < -90))
    {
        ROS_DEBUG_STREAM("   LATLON_2_XY: Latitude must be between -90 and 90");
        return false;
    }

    if ((req.longitude > 180) || (req.longitude < -180))
    {
        ROS_DEBUG_STREAM("   LATLON_2_XY: Longitude must be between -180 and 180");
        return false;
    }

    Xy coords = latlon2xy_helper(req.latitude, req.longitude);
    resp.x = coords.x;
    resp.y = coords.y;
    return true;
}


bool xy_2_latlon(ira_open_street_map::xy_2_latlon::Request& req, ira_open_street_map::xy_2_latlon::Response& resp)
{
    Coordinates coords = xy2latlon_helper(req.x, req.y, 32, false);
    resp.latitude = coords.latitude;
    resp.longitude = coords.longitude;
    return true;
}

bool ecef_2_lla(ira_open_street_map::ecef_2_lla::Request& req, ira_open_street_map::ecef_2_lla::Response& resp)
{
    Coordinates coords = ecef2lla_helper(req.x, req.y, req.z);
    resp.latitude = coords.latitude;
    resp.longitude = coords.longitude;
    resp.altitude = coords.altitude;
    return true;
}

bool lla_2_ecef(ira_open_street_map::lla_2_ecef::Request& req, ira_open_street_map::lla_2_ecef::Response& resp)
{
    geometry_msgs::Point p = lla2ecef_helper(req.latitude, req.longitude, req.altitude);
    resp.x = p.x;
    resp.y = p.y;
    resp.z = p.z;
    return true;
}

/**
 * @brief get_distance_from_xy
 * @param req
 * @param resp
 * @return
 */
bool get_distance_from_xy(ira_open_street_map::get_distance_from_xy::Request& req, ira_open_street_map::get_distance_from_xy::Response& resp)
{

    // Calculate distance between request node and current way node (Sqrt((N1-N2)²+(E1-E2)²))
    resp.distance = get_distance_helper(req.x1, req.y1, req.x2, req.y2); // (m)

    return true;
}

/**
 * @brief getDistanceFromLaneCenter
 * @param req
 * @param resp
 * @return the position of the center of the requested lane, in XY coordinate frame
 *
 *              |    .    |    .    |
 *              |    .    |    .    |
 *              |    .    |    .    |
 *              |    .    |    .    |
 *              |    .    |    .    |
 *              |    .    |    .    |
 *                *
 *                <--> this is the distance this function is going to return
 *
 * refs #435, #527
 *
 * vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
 * The function is set to DEPRECATED as a warning, beacause it is not finished
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 */
ROS_DEPRECATED bool getDistanceFromLaneCenter(ira_open_street_map::getDistanceFromLaneCenter::Request& req, ira_open_street_map::getDistanceFromLaneCenter::Response& resp)
{
    const char* search_tag;
    int         number_of_lanes = 0;
    double      width = 0.0f;           ///< value from OSM
    int         oneway = 0;             ///< boolean oneway, i need only to know if is oneway or not, reversed is still oneway, so 1 oneway, 0 bothway


    ROS_DEBUG_STREAM("getLaneCenter for way_id: " << req.way_id);

    for (std::set<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = oh.m_ways.begin(); way_itr != oh.m_ways.end(); way_itr++)
    {
        ROS_DEBUG_STREAM((*way_itr)->id());

        // Get way-node list
        if ((*way_itr)->id() == req.way_id)
        {
            search_tag = (*way_itr)->tags().get_value_by_key("highway");
            if (!search_tag)
                continue;

            search_tag = (*way_itr)->tags().get_value_by_key("lanes");
            if (!search_tag)
                number_of_lanes = 1;
            else
                number_of_lanes = atoi(search_tag);

            search_tag = (*way_itr)->tags().get_value_by_key("width");
            if (!search_tag)
                width = 0;
            else
                width = boost::lexical_cast<double>(search_tag);

            search_tag = (*way_itr)->tags().get_value_by_key("oneway");
            if (!search_tag)
                oneway = 0;
            else
            {
                if (strcmp(search_tag, "yes") == 0 || strcmp(search_tag, "true") == 0 || strcmp(search_tag, "1") == 0)
                {
                    oneway = 1; // it is oneway
                }
                else if (strcmp(search_tag, "no") == 0 || strcmp(search_tag, "false") == 0 || strcmp(search_tag, "0") == 0)
                {
                    oneway = 0; // it is no-oneway
                }
                else if (strcmp(search_tag, "-1") == 0 || strcmp(search_tag, "reverse") == 0)
                {
                    oneway = 1; // it is oneway, but in reverse order
                }
            }

            //calculate shortest distance between particle and every way's segments (a way segment is defined by 2 nodes)
            Xy node_1, node_2, snap, snapped_xy, min_dist_node1, min_dist_node2;
            Osmium::OSM::WayNodeList::iterator it2;
            shared_ptr<Osmium::OSM::Way const> min_distance_way;
            double lat, lat2, lon, lon2, dist = 0.0f;
            double snapped_min_distance = 999999999;
            int64 way_id = 0;
            Xy particle = {req.x, req.y};

            Osmium::OSM::WayNodeList way_n_list = (*way_itr)->nodes();
            for (Osmium::OSM::WayNodeList::iterator node_list_itr = way_n_list.begin(); node_list_itr != --way_n_list.end(); node_list_itr++ )
            {
                // extract 1st node coordinates
                lat = node_list_itr->position().lat();
                lon = node_list_itr->position().lon();
                node_1 = latlon2xy_helper(lat, lon);

                // extract 2nd node coordinates
                it2 = node_list_itr;
                it2++;
                lat2 = it2->position().lat();
                lon2 = it2->position().lon();
                node_2 = latlon2xy_helper(lat2, lon2);

                // snap particle (on the way=highway segment defined by node_1 and node_2)
                snap = snap_particle_helper(node_1, node_2, particle);

                // calculate distance from particle to snapped way segment
                dist = get_distance_helper(snap.x, snap.y, particle.x, particle.y);

                if (dist < snapped_min_distance)
                {
                    // update min distance
                    snapped_min_distance = dist;
                    min_distance_way = *way_itr;
                    snapped_xy = snap;
                    min_dist_node1 = node_1;
                    min_dist_node2 = node_2;
                    way_id = (*way_itr)->id();
                }

                // break if it's last way segment
                if ( it2 == way_n_list.end() )
                {
                    break;
                }
            }

            // calculates the direction of the hypotheses based on the two points. needs the way-ptr to check
            // whether the way is oneway or not
            Way_dir_struct direction = way_direction_helper(min_distance_way, min_dist_node1, min_dist_node2);

            // Set response values
            resp.snapped_x       = snapped_xy.x;
            resp.snapped_y       = snapped_xy.y;
            resp.way_dir_rad     = direction.yaw_rad;
            resp.way_dir_degrees = direction.yaw_deg;
            resp.way_dir_quat_w  = direction.q_w;
            resp.way_dir_quat_x  = direction.q_x;
            resp.way_dir_quat_y  = direction.q_y;
            resp.way_dir_quat_z  = direction.q_z;

            resp.distance_from_way_center = snapped_min_distance; // this is the distance from the Hightway/Way center
            resp.way_id = way_id;


            // Distance from the lane center, lanes are equivalently distributed, i.e. max_width/n_lanes
            resp.distance_from_lane_center =  abs( ((width / 2.0f) - snapped_min_distance) - (width / (2.0f * number_of_lanes)) );


            ROS_DEBUG_STREAM("getLaneCenter ACK");
            return true;
        }
    }

    ROS_WARN_STREAM("Way-id: " << req.way_id <<  " not found in the current map");
    return false;


//    // Get snapped particle direction using the min_distance segment of the nearest way=highway element
//    Way_dir_struct dir = way_direction_helper(min_distance_way, min_dist_node1, min_dist_node2);

//    // Set response values
//    resp.snapped_x = snapped_xy.x;
//    resp.snapped_y = snapped_xy.y;
//    resp.way_dir_rad = dir.yaw_rad;
//    resp.way_dir_degrees = dir.yaw_deg;
//    resp.way_dir_quat_w = dir.q_w;
//    resp.way_dir_quat_x = dir.q_x;
//    resp.way_dir_quat_y = dir.q_y;
//    resp.way_dir_quat_z = dir.q_z;
//    resp.way_dir_opposite_particles = dir.opposite_direction;
//    resp.distance_from_way = min_distance;
//    resp.way_id=way_id;

//    return true;

}




/**
 * @brief get_node_coordinates
 * @param req
 * @param resp
 * @return
 */
bool getHighwayInfo(ira_open_street_map::getHighwayInfo::Request& req, ira_open_street_map::getHighwayInfo::Response& resp)
{
    /*
     *  REQ:    way_id
     *  REPS:   n. of lanes
     *          road width
     *          oneway tag
     */

    const char* search_tag;
    ROS_DEBUG_STREAM("Searching way_id " << req.way_id);
    for (std::set<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = oh.m_ways.begin(); way_itr != oh.m_ways.end(); way_itr++)
    {
        ROS_DEBUG_STREAM((*way_itr)->id());
        // Get way-node list
        if ((*way_itr)->id() == req.way_id)
        {
            search_tag = (*way_itr)->tags().get_value_by_key("highway");
            if (!search_tag)
                continue;

            search_tag = (*way_itr)->tags().get_value_by_key("lanes");
            if (!search_tag)
                resp.number_of_lanes = 0;
            else
                resp.number_of_lanes = atoi(search_tag);

            search_tag = (*way_itr)->tags().get_value_by_key("width");
            if (!search_tag)
                resp.width = 0;
            else
                resp.width = boost::lexical_cast<double>(search_tag);

            search_tag = (*way_itr)->tags().get_value_by_key("oneway");
            if (!search_tag)
                resp.oneway = 0;
            else
            {
                if (strcmp(search_tag, "yes") == 0 || strcmp(search_tag, "true") == 0 || strcmp(search_tag, "1") == 0)
                {
                    resp.oneway = 1;
                }
                else if (strcmp(search_tag, "no") == 0 || strcmp(search_tag, "false") == 0 || strcmp(search_tag, "0") == 0)
                {
                    resp.oneway = 0;
                }
                else if (strcmp(search_tag, "-1") == 0 || strcmp(search_tag, "reverse") == 0)
                {
                    resp.oneway = -1;
                }
            }

            ROS_DEBUG_STREAM("getHighwayInfo ACK");
            return true;
        }
    }
    ROS_WARN_STREAM("Way-id: " << req.way_id <<  " not found in the current map");
    return false;
}


bool get_node_coordinates(ira_open_street_map::get_node_coordinates::Request& req, ira_open_street_map::get_node_coordinates::Response& resp)
{

    ROS_DEBUG_STREAM("SERVICE get_node_coordinates:");

    // Cycle through every stored way
    for (std::set<shared_ptr<Osmium::OSM::Node const> >::iterator node_itr = oh.m_nodes.begin(); node_itr != oh.m_nodes.end(); node_itr++)
    {
        if ( (*node_itr)->id() == req.node_id  )
        {
            double lat = (*node_itr)->position().lat();
            double lon = (*node_itr)->position().lon();
            Xy coords = latlon2xy_helper(lat, lon);


            ROS_DEBUG_STREAM("   Latitude, Longitude (WGS84):");
            ROS_DEBUG_STREAM("   " << lat  << " " << lon );

            ROS_DEBUG_STREAM("   X, Y (UTM):");
            ROS_DEBUG_STREAM("   " << coords.x << " " << coords.y);

            resp.x = coords.x;
            resp.y = coords.y;
            resp.latitude = lat;
            resp.longitude = lon;
            return true;
        }
    }

    ROS_DEBUG_STREAM("   Node ID not found inside map");
    return true;
}

visualization_msgs::Marker buildings;
/**
 * @brief publish_waylist
 * @param req
 * @param resp
 * @return
 */
void load_buildinglist()
{
    buildings.header.frame_id = "map";
    buildings.header.stamp = ros::Time();
    buildings.ns = "buildings";
    buildings.id = 3;
    buildings.type = visualization_msgs::Marker::TRIANGLE_LIST;
    buildings.action = visualization_msgs::Marker::ADD;
    buildings.scale.x = 1.0;
    buildings.scale.y = 1.0;
    buildings.scale.z = 1.0;
    buildings.color.r = 0.0f;
    buildings.color.g = 0.9f;
    buildings.color.b = 0.7f;
    buildings.color.a = 0.75f;

    Xy coords_A, coords_B;
    float height = 3.0f;
    geometry_msgs::Point A, h_A, B, h_B;

    for(std::set<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = oh.m_ways.begin(); way_itr != oh.m_ways.end(); way_itr++)
    {
        // Get way-node list
        Osmium::OSM::WayNodeList waylist = (*way_itr)->nodes();

        // Use only ways with Key:building
        const char* building_tag = (*way_itr)->tags().get_value_by_key("building");

        if(!building_tag)
            continue;

        for(Osmium::OSM::WayNodeList::iterator node_list_itr = waylist.begin(); node_list_itr != waylist.end() - 1; node_list_itr++ )
        {
            // Get way-node coordinates
            coords_A = latlon2xy_helper(node_list_itr->position().lat(),node_list_itr->position().lon());
            coords_B = latlon2xy_helper((node_list_itr + 1)->position().lat(),(node_list_itr + 1)->position().lon());

            A.x = coords_A.x;
            A.y = coords_A.y;
            A.z = 0.0;

            h_A.x = A.x;
            h_A.y = A.y;
            h_A.z = A.z + height;

            B.x = coords_B.x;
            B.y = coords_B.y;
            B.z = 0.0;

            h_B.x = B.x;
            h_B.y = B.y;
            h_B.z = B.z + height;

            buildings.points.push_back(A);
            buildings.points.push_back(h_A);
            buildings.points.push_back(B);
            buildings.points.push_back(B);
            buildings.points.push_back(h_A);
            buildings.points.push_back(h_B);
        }
    }
}


visualization_msgs::MarkerArray waylistArray_oneway;
visualization_msgs::MarkerArray waylistArray;
visualization_msgs::MarkerArray nodelistArray;
visualization_msgs::MarkerArray directionArray;
/**
 * @brief publish_waylist
 * @param req
 * @param resp
 * @return
 */
void load_waylist()
{
    visualization_msgs::Marker way_single_part;
    way_single_part.header.frame_id = "map";
    way_single_part.header.stamp = ros::Time::now();
    way_single_part.ns = "way";
    way_single_part.id = 0;
    way_single_part.type = visualization_msgs::Marker::LINE_LIST;
    way_single_part.action = visualization_msgs::Marker::ADD;
    way_single_part.pose.position.x = 0;
    way_single_part.pose.position.y = 0;
    way_single_part.pose.position.z = 0;
    way_single_part.pose.orientation.x = 0.0;
    way_single_part.pose.orientation.y = 0.0;
    way_single_part.pose.orientation.z = 0.0;
    way_single_part.pose.orientation.w = 1.0;
    way_single_part.scale.x = 1;    //WAY Key:highway 1m fixed width
    way_single_part.scale.y = 0;    //don't used
    way_single_part.scale.z = 0;    //don't used
    way_single_part.color.a = 1.0; // Don't forget to set the alpha!
    way_single_part.color.r = 1.0;
    way_single_part.color.g = 1.0;
    way_single_part.color.b = 1.0;

    visualization_msgs::Marker node;
    node.header.frame_id = "map";
    node.header.stamp = ros::Time::now();
    node.ns = "node";
    node.id = 1;
    node.type = visualization_msgs::Marker::CYLINDER;
    node.action = visualization_msgs::Marker::ADD;
    node.pose.position.x = 0;
    node.pose.position.y = 0;
    node.pose.position.z = 0;
    node.pose.orientation.x = 0.0;
    node.pose.orientation.y = 0.0;
    node.pose.orientation.z = 0.0;
    node.pose.orientation.w = 1.0;
    node.scale.x = 2;
    node.scale.y = 2;
    node.scale.z = 0.4;
    node.color.a = 1.0; // Don't forget to set the alpha!
    node.color.r = 1.0;
    node.color.g = 1.0;
    node.color.b = 0.0;

    visualization_msgs::Marker direction;
    direction.header.frame_id = "map";
    direction.header.stamp = ros::Time::now();
    direction.ns = "direction";
    direction.id = 2;
    direction.type = visualization_msgs::Marker::ARROW;
    direction.action = visualization_msgs::Marker::ADD;
    direction.pose.position.x = 0;
    direction.pose.position.y = 0;
    direction.pose.position.z = 0;
    direction.pose.orientation.x = 0.0;
    direction.pose.orientation.y = 0.0;
    direction.pose.orientation.z = 0.0;
    direction.pose.orientation.w = 1.0;
    direction.scale.x = 2;
    direction.scale.y = 0.5;
    direction.scale.z = 0.2;
    direction.color.a = 1.0; // Don't forget to set the alpha!
    direction.color.r = 0.0;
    direction.color.g = 1.0;
    direction.color.b = 1.0;


    Xy coords;
    unsigned int id = 0, id2 = 0, id3 = 0;
    geometry_msgs::Point point, prev_point;
    bool got_only_one_point = true;

    waylistArray.markers.clear();
    waylistArray_oneway.markers.clear();

    for (std::set<shared_ptr<Osmium::OSM::Way const> >::iterator way_itr = oh.m_ways.begin(); way_itr != oh.m_ways.end(); way_itr++)
    {
        // Get way-node list
        Osmium::OSM::WayNodeList waylist = (*way_itr)->nodes();

        // Use only ways with Key:Highway
        const char* highway = (*way_itr)->tags().get_value_by_key("highway");
        if (!highway)
            continue;

        const char* one_way = (*way_itr)->tags().get_value_by_key("oneway");
        // ONEWAY TAG FOUND:
        if ( one_way )
        {
            if (strcmp(one_way, "yes") == 0 || strcmp(one_way, "true") == 0 || strcmp(one_way, "1") == 0)
            {
                way_single_part.color.r = 0.0;
                way_single_part.color.g = 1.0;
                way_single_part.color.b = 0.0;
            }
            else if (strcmp(one_way, "no") == 0 || strcmp(one_way, "false") == 0 || strcmp(one_way, "0") == 0)
            {
                way_single_part.color.r = 1.0;
                way_single_part.color.g = 0.0;
                way_single_part.color.b = 0.0;
            }
            else if (strcmp(one_way, "-1") == 0 || strcmp(one_way, "reverse") == 0)
            {
                way_single_part.color.r = 0.0;
                way_single_part.color.g = 1.0;
                way_single_part.color.b = 0.0;
            }
        }
        // NO TAGS FOUND!
        else
        {
            way_single_part.color.r = 1.0;
            way_single_part.color.g = 0.0;
            way_single_part.color.b = 0.0;
        }


        for (Osmium::OSM::WayNodeList::iterator node_list_itr = waylist.begin(); node_list_itr != waylist.end(); node_list_itr++ )
        {
            // Get way-node coordinates
            coords = latlon2xy_helper(node_list_itr->position().lat(), node_list_itr->position().lon());
            node.pose.position.x = point.x = coords.x;
            node.pose.position.y = point.y = coords.y;
            node.pose.position.z = point.z = 0;

            node.id = id++;
            node.header.seq = id;
            node.pose.position.z = 0.2;
            nodelistArray.markers.push_back(node);

            if (got_only_one_point)
            {
                prev_point = point;
                got_only_one_point = false;
            }
            else
            {
                double middle_x, middle_y, theta;
                tf::Quaternion q;

                if ( one_way )
                {
                    if (strcmp(one_way, "yes") == 0 || strcmp(one_way, "true") == 0 || strcmp(one_way, "1") == 0)
                    {
                        middle_x = (prev_point.x + point.x) / 2;
                        middle_y = (prev_point.y + point.y) / 2;
                        theta = atan2(point.y - prev_point.y, point.x - prev_point.x);
                        q = tf::createQuaternionFromYaw(theta);
                        direction.pose.position.x = middle_x + 2;
                        direction.pose.position.y = middle_y + 2;
                        direction.pose.position.z = 0;
                        direction.pose.orientation.x = q.x();
                        direction.pose.orientation.y = q.y();
                        direction.pose.orientation.z = q.z();
                        direction.pose.orientation.w = q.w();
                        direction.id = id3++;
                        direction.header.seq = id3;
                        directionArray.markers.push_back(direction);
                    }
                    else if (strcmp(one_way, "no") == 0 || strcmp(one_way, "false") == 0 || strcmp(one_way, "0") == 0)
                    {
                        middle_x = (prev_point.x + point.x) / 2;
                        middle_y = (prev_point.y + point.y) / 2;
                        theta = atan2(prev_point.y - point.y, prev_point.x - point.x);
                        q = tf::createQuaternionFromYaw(theta);
                        direction.pose.position.x = middle_x + 2;
                        direction.pose.position.y = middle_y + 2;
                        direction.pose.position.z = 0;
                        direction.pose.orientation.x = q.x();
                        direction.pose.orientation.y = q.y();
                        direction.pose.orientation.z = q.z();
                        direction.pose.orientation.w = q.w();
                        direction.id = id3++;
                        direction.header.seq = id3;
                        directionArray.markers.push_back(direction);

                        middle_x = (prev_point.x + point.x) / 2;
                        middle_y = (prev_point.y + point.y) / 2;
                        theta = atan2(point.y - prev_point.y, point.x - prev_point.x);
                        q = tf::createQuaternionFromYaw(theta);
                        direction.pose.position.x = middle_x - 2;
                        direction.pose.position.y = middle_y - 2;
                        direction.pose.position.z = 0;
                        direction.pose.orientation.x = q.x();
                        direction.pose.orientation.y = q.y();
                        direction.pose.orientation.z = q.z();
                        direction.pose.orientation.w = q.w();
                        direction.id = id3++;
                        direction.header.seq = id3;
                        directionArray.markers.push_back(direction);
                    }
                    else if (strcmp(one_way, "-1") == 0 || strcmp(one_way, "reverse") == 0)
                    {
                        middle_x = (prev_point.x + point.x) / 2;
                        middle_y = (prev_point.y + point.y) / 2;
                        theta = atan2(point.y - prev_point.y, point.x - prev_point.x);
                        q = tf::createQuaternionFromYaw(theta);
                        direction.pose.position.x = middle_x + 2;
                        direction.pose.position.y = middle_y + 2;
                        direction.pose.position.z = 0;
                        direction.pose.orientation.x = q.x();
                        direction.pose.orientation.y = q.y();
                        direction.pose.orientation.z = q.z();
                        direction.pose.orientation.w = q.w();
                        direction.id = id3++;
                        direction.header.seq = id3;
                        directionArray.markers.push_back(direction);
                    }
                }
                // NO TAGS FOUND!
                else
                {
                    middle_x = (prev_point.x + point.x) / 2;
                    middle_y = (prev_point.y + point.y) / 2;
                    theta = atan2(prev_point.y - point.y, prev_point.x - point.x);
                    q = tf::createQuaternionFromYaw(theta);
                    direction.pose.position.x = middle_x + 2;
                    direction.pose.position.y = middle_y + 2;
                    direction.pose.position.z = 0;
                    direction.pose.orientation.x = q.x();
                    direction.pose.orientation.y = q.y();
                    direction.pose.orientation.z = q.z();
                    direction.pose.orientation.w = q.w();
                    direction.id = id3++;
                    direction.header.seq = id3;
                    directionArray.markers.push_back(direction);

                    middle_x = (prev_point.x + point.x) / 2;
                    middle_y = (prev_point.y + point.y) / 2;
                    theta = atan2(point.y - prev_point.y, point.x - prev_point.x);
                    q = tf::createQuaternionFromYaw(theta);
                    direction.pose.position.x = middle_x - 2;
                    direction.pose.position.y = middle_y - 2;
                    direction.pose.position.z = 0;
                    direction.pose.orientation.x = q.x();
                    direction.pose.orientation.y = q.y();
                    direction.pose.orientation.z = q.z();
                    direction.pose.orientation.w = q.w();
                    direction.id = id3++;
                    direction.header.seq = id3;
                    directionArray.markers.push_back(direction);
                }


                way_single_part.points.push_back(prev_point);
                way_single_part.points.push_back(point);
                prev_point = point;
            }
        }
        way_single_part.id = id2++;
        way_single_part.header.seq = id2;
        got_only_one_point = true;

        if ( one_way )
        {
            if (strcmp(one_way, "no") == 0 || strcmp(one_way, "false") == 0 || strcmp(one_way, "0") == 0)
            {
                waylistArray.markers.push_back(way_single_part);
            }
            else
            {
                waylistArray_oneway.markers.push_back(way_single_part);
            }
        }
        // NO TAGS FOUND!
        else
        {
            waylistArray.markers.push_back(way_single_part);
        }

        way_single_part.points.clear();

    } // finite le vie
}

/* ================================================== */

int main(int argc, char* argv[])
{

    // init node
    ros::init(argc, argv, "osm_query_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    ros::Publisher markerArrayPublisher_ways = nh.advertise<visualization_msgs::MarkerArray>("/ways", 5);
    ros::Publisher markerPublisher_buildings = nh.advertise<visualization_msgs::Marker>("/buildings", 1);

    ROS_INFO_STREAM("OSM QUERY NODE STARTED:");

    // check filepath from arguments
    stringstream map_path;
    if (argc != 2)
    {
        map_path << ros::package::getPath("ira_open_street_map") << "/maps/kitti_05_debug3.osm";
    }
    else
    {
        // read .osm file
        map_path << argv[1];
    }


    // init OSM file
    ROS_INFO_STREAM("Trying to open" << map_path.str().c_str());
    Osmium::OSMFile infile(map_path.str().c_str());

    // Read OSM with ObjestHandler which is a combination of ObjectStore and CoordinateForWays handlers
    time_t t0 = time(NULL);
    ROS_INFO_STREAM("   Reading: '" << infile.filename().c_str() << "'");
    Osmium::Input::read(infile, oh);
    ROS_INFO_STREAM("   Finished reading OSM map file (time spent: " << time(NULL) - t0 << "s)");
    ROS_INFO_STREAM("   Nodes: " << oh.m_nodes.size() << "  Ways: " << oh.m_ways.size() << "  Relations: " << oh.m_relations.size());

    // init services
    ros::ServiceServer server_lla2ecef                  = nh.advertiseService("/ira_open_street_map/lla_2_ecef"                  , &lla_2_ecef);
    ros::ServiceServer server_ecef2lla                  = nh.advertiseService("/ira_open_street_map/ecef_2_lla"                  , &ecef_2_lla);
    ros::ServiceServer server_latlon2xy                 = nh.advertiseService("/ira_open_street_map/latlon_2_xy"                 , &latlon_2_xy);
    ros::ServiceServer server_xy2latlon                 = nh.advertiseService("/ira_open_street_map/xy_2_latlon"                 , &xy_2_latlon);
    ros::ServiceServer server_snapparticle              = nh.advertiseService("/ira_open_street_map/snap_particle_xy"            , &snap_particle_xy);
    ros::ServiceServer server_waydirection              = nh.advertiseService("/ira_open_street_map/way_direction"               , &way_direction);
    ros::ServiceServer server_nodecoords                = nh.advertiseService("/ira_open_street_map/get_node_coordinates"        , &get_node_coordinates);
    ros::ServiceServer server_get_distance_from_way     = nh.advertiseService("/ira_open_street_map/get_closest_way_distance_utm", &get_closest_way_distance_utm);
    ros::ServiceServer server_distance_xy               = nh.advertiseService("/ira_open_street_map/get_distance_from_xy"        , &get_distance_from_xy);

    ros::ServiceServer server_getHighwayInfo            = nh.advertiseService("/ira_open_street_map/getHighwayInfo"              , &getHighwayInfo);
    ros::ServiceServer server_getDistanceFromLaneCenter = nh.advertiseService("/ira_open_street_map/getDistanceFromLaneCenter"   , &getDistanceFromLaneCenter);
    ros::ServiceServer server_oneway                    = nh.advertiseService("/ira_open_street_map/oneWay"                      , &getOneWayInfo);

    // test service call
    //    ira_open_street_map:markerArrayPublisher_ways:is_valid_location::Request test_req;
    //    ira_open_street_map::is_valid_location::Response test_resp;
    //    test_req.latitude = 45.5209287;
    //    test_req.longitude = 9.2163431;
    //    test_req.max_distance_radius = 500;const char* one_way = way->tags().get_value_by_key("oneway");
    //    is_valid_location(test_req, test_resp);

    // test service call
    //    ira_open_street_map::is_valid_location_xy::Request test_req;
    //    ira_open_street_map::is_valid_location_xy::Response test_resp;
    //    test_req.x = 516872.3;
    //    test_req.y = 5041030.3;
    //    test_req.max_distance_radius = 100;
    //    is_valid_location_xy(test_req, test_resp);

    // test lat/lon conversion
    //    Xy coords = latlon_converter(45.5238,9.21954);
    //    Coordinates coords = xy2latlon_helper(511791.4,4999624.7,32,false);

    //    Xy A = {517219.5, 5041125.2};
    //    Xy B = {517092.2, 5041194.1};
    //    Xy C = {517152.9, 5041172.2};
    //    Xy D = snapxy2way_outbounds_helper(A, B, C);

    load_waylist();
    load_buildinglist();

    markerArrayPublisher_ways.publish(waylistArray);
    markerArrayPublisher_ways.publish(waylistArray_oneway);
    markerArrayPublisher_ways.publish(nodelistArray);
    markerArrayPublisher_ways.publish(directionArray);
    markerPublisher_buildings.publish(buildings);
    // Multithread Spinner
    spinner.start();

    ros::Duration sleepTime(1);
    while (ros::ok())
    {
        markerArrayPublisher_ways.publish(waylistArray);
        markerArrayPublisher_ways.publish(waylistArray_oneway);
        markerArrayPublisher_ways.publish(nodelistArray);
        markerArrayPublisher_ways.publish(directionArray);
        markerPublisher_buildings.publish(buildings);
        sleepTime.sleep();
    }

    ros::waitForShutdown();


    google::protobuf::ShutdownProtobufLibrary();
}
