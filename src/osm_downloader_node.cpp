/***************************************************************************
 *                                                                         *
 *   IRALab - Informatics & Robotics for Automation Laboratory             *
 *      Universita' degli Studi Milano - Bicocca, DISCO                    *
 *      Building U14, viale Sarca 336, 20126, Milano, Italy                *
 *                                                                         *
 *   Author:    Dario Limongi                                              *
 *   Email:     dario.limongi@gmail.com                                    *
 *   Date:      11/20/2014                                                 *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <curl/curl.h>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>

using namespace std;

/**
 * Download webpage located at "url" parameter and save it at "file_path"
 * @param url
 * @param file_path
 */
void get_page(const char* url, const char* file_path)
{
    // Open cURL
    CURL* easyhandle = curl_easy_init();

    if(easyhandle)
    {
        // Set URL
        curl_easy_setopt( easyhandle, CURLOPT_URL, url ) ;

        // Open file
        FILE* file = fopen( file_path, "w");

        // Set writing opt, and write URL on file
        curl_easy_setopt( easyhandle, CURLOPT_WRITEDATA, file) ;
        curl_easy_perform( easyhandle );

        // Close file
        fclose(file);
    }

    // Close cURL
    curl_easy_cleanup( easyhandle );
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "osm_downloader_node");
    ros::NodeHandle nh;

    // Map coordinates (default coordinates of U14 - Bicocca Computer Science department building)
    double lat = 45.523774;
    double lon =  9.219543;

    // Offsets (m)
    double dN = 500;
    double dE = 500;

    // Read args
    if(argc > 1){
        // Download a square map centered in <latitude, longitude> with <offset> width
        if(argc == 4){
            // Read latitude arg
            double arg_lat = atof(argv[1]);
            if( (arg_lat > 90) || (arg_lat < -90)){
                ROS_INFO_STREAM("Latitude must be between -90 and 90. Value given: " << arg_lat);
                return -1;
            }
            lat = arg_lat;

            // Read longitude arg
            double arg_lon = atof(argv[2]);
            if( (arg_lon > 180) || (arg_lon < -180)){
                ROS_INFO_STREAM("Longitude must be between -180 and 180. Value given: " << arg_lon);
                return -1;
            }
            lon = arg_lon;

            // Read map offset
            double offset = atof(argv[3]);
            if( offset <= 0 || offset >= 2000 ){
                ROS_INFO_STREAM("Offset must be positive and less then 2km (OpenStreetMap server slow down prevenction). Value given: " << offset);
                return -1;
            }
            dN = offset; dE = offset;
        }
        // Download a map with the given bounding box coordinates
        else if(argc == 5){

        }
        else{
            ROS_INFO_STREAM("CUSTOM ARGS USAGE:");
            ROS_INFO_STREAM("   Node args: <latitude> <longitude> <map_offset_meters>");
            ROS_INFO_STREAM("   Node args: <box_left_longitude> <box_bottom_latitude> <box_right_longitude> <tbox_top_latitude>");
        }
    }
    else
    {
        ROS_INFO_STREAM("Running node with no args, downloading default map");
        ROS_INFO_STREAM("CUSTOM ARGS USAGE:");
        ROS_INFO_STREAM("   Node args: <latitude> <longitude> <map_offset_meters>");
        ROS_INFO_STREAM("   Node args: <box_left_longitude> <box_bottom_latitude> <box_right_longitude> <tbox_top_latitude>");
    }
    // Earth’s radius (m)
    double R = 6378137;

    // Left bounding box coordinate
    double left_rad; // radians
    double left_deg; // decimal degrees

    // Bottom bounding box coordinate
    double bottom_rad;
    double bottom_deg;

    // Right bounding box coordinate
    double right_rad; // radians
    double right_deg; // decimal degrees

    // Top bounding box coordinate
    double top_rad;
    double top_deg;

    ROS_INFO_STREAM("OSM MAP DOWNLOADER LAUNCHED");
    if(argc == 5){
        // KITTI 01 Bounding box:
        //
        // left: 8.4633     (lat)
        // bottom: 49.0055  (lon)
        // right: 8.4903    (lat)
        // top: 49.0184     (lon)
        //
        // rosrun ira_open_street_map osm_downloader_node 8.4633 49.0055 8.4903 49.0184
        left_deg = strtod(argv[1], (char **) NULL);
        bottom_deg = strtod(argv[2], (char **) NULL);
        right_deg = strtod(argv[3], (char **) NULL);
        top_deg = strtod(argv[4], (char **) NULL);
        ROS_INFO_STREAM("   Bounding box");
        ROS_INFO_STREAM("       left_latitude: " << left_deg << " bottom_longitude: " << bottom_deg << " right_latitude: " << right_deg << " top_longitude: " << top_deg);
    }
    else{
        ROS_INFO_STREAM("   Latitude: " << lat << " Longitude: " << lon << " Offset: " << dE);

        // Earth’s radius (m)
        R = 6378137;

        // Left bounding box coordinate
        left_rad = (-dE) / ( R * cos(M_PI * lat/180) ); // radians
        left_deg = lon + (left_rad * 180/M_PI); // decimal degrees

        // Bottom bounding box coordinate
        bottom_rad = (-dN) / R;
        bottom_deg = lat + (bottom_rad * 180/M_PI);

        // Right bounding box coordinate
        right_rad = dE / ( R * cos(M_PI * lat/180) ); // radians
        right_deg = lon + (right_rad * 180/M_PI); // decimal degrees

        // Top bounding box coordinate
        top_rad = dN / R;
        top_deg = lat + (top_rad * 180/M_PI);
    }

    // OpenStreetMap URL + export map API parameters
    //  left    => is the longitude of the left (westernmost) side of the bounding box.
    //  bottom  => is the latitude of the bottom (southernmost) side of the bounding box.
    //  right   => is the longitude of the right (easternmost) side of the bounding box.
    //  top     => is the latitude of the top (northernmost) side of the bounding box.
    stringstream osm_api_url;
//    osm_api_url << "https://www.openstreetmap.org/api/0.6/map?bbox=" << left_deg << "," << bottom_deg << "," << right_deg << "," << top_deg;
    osm_api_url << "http://www.overpass-api.de/api/xapi?*[highway=*][bbox=" << left_deg << "," << bottom_deg << "," << right_deg << "," << top_deg <<"]";

//    cout << osm_api_url.str().c_str() << endl;

    // Save map inside "package:/ira_open_street_map/maps/" directory with "map.osm" filename
    stringstream map_path;
    map_path << ros::package::getPath("ira_open_street_map") << "/maps/map.osm";

    ROS_INFO_STREAM("Left: " << left_deg << " Bottom: " << bottom_deg << " Right: " << right_deg << " Top: " << top_deg);
    ROS_INFO_STREAM("Downloading...");
    get_page( osm_api_url.str().c_str(), map_path.str().c_str() );
    ROS_INFO_STREAM("Download completed." << "File saved in: " << map_path.str().c_str());
    return 0;
}
