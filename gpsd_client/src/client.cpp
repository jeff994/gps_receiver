#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <libgpsmm.h>
//-------------------------------------------------------------------------------------------
#include <stdlib.h> //added by cheng16/8
//-------------------------------------------------------------------------------------------

using namespace gps_common;
using namespace sensor_msgs;

class GPSDClient {
  public:
    GPSDClient() : privnode("~"), gps(NULL), use_gps_time(true), check_fix_by_variance(true) {}

//----added by cheng 16/8-------------------------------------------------------------------------------
    void run_gpsd() {
      std::string host = "localhost";
      int port = 2947;
      std::string dev = "None";
      privnode.getParam("host", host);
      privnode.getParam("port", port);
      privnode.getParam("dev", dev);
      // ROS_INFO("run %d", port);
      std::stringstream run_gpsd;
      run_gpsd << "gpsd -S " << port << " " << dev;
      ROS_INFO("%s", run_gpsd.str().c_str());
      system(run_gpsd.str().c_str());
    }
//------------------------------------------------------------------------------------------------------

    bool start() {
      gps_fix_pub = node.advertise<GPSFix>("extended_fix", 1);
      navsat_fix_pub = node.advertise<NavSatFix>("fix", 1);

      privnode.getParam("use_gps_time", use_gps_time);
      privnode.getParam("check_fix_by_variance", check_fix_by_variance);

      std::string host = "localhost";
      int port = 2947;
      privnode.getParam("host", host);
      privnode.getParam("port", port);

//----added by cheng 16/8-------------------------------------------------------------------------------
      
      std::string dev = "None";
      privnode.getParam("dev", dev);

//------------------------------------------------------------------------------------------------------

      char port_s[12];
      snprintf(port_s, 12, "%d", port);

      gps_data_t *resp = NULL;

#if GPSD_API_MAJOR_VERSION >= 5
      gps = new gpsmm(host.c_str(), port_s);
      resp = gps->stream(WATCH_ENABLE);
#elif GPSD_API_MAJOR_VERSION == 4
      gps = new gpsmm();
      gps->open(host.c_str(), port_s);
      resp = gps->stream(WATCH_ENABLE);
#else
      gps = new gpsmm();
      resp = gps->open(host.c_str(), port_s);
      gps->query("w\n");
#endif

//-------added by cheng16/8--------------------------------------------------------------------
      privnode.deleteParam("host");
      privnode.deleteParam("port");
      privnode.deleteParam("dev");
//---------------------------------------------------------------------------------------------

      if (resp == NULL || dev == "None") {
        ROS_ERROR("Failed to open GPSd");
        return false;
      }

      ROS_INFO("GPSd opened");
      return true;
    }

    void step() {
      // ROS_INFO("step");
#if GPSD_API_MAJOR_VERSION >= 5
      if (!gps->waiting(1e6)) {
        ROS_WARN("GPS wait time more than 1s .. : !gps->waiting(1e6)");
        return;
      }

      // ROS_INFO("p = gps->read()");
      gps_data_t *p = gps->read();
      
#else
      // ROS_INFO("p = gps->poll()");
      gps_data_t *p = gps->poll();
      
#endif
      // ROS_INFO("process data");
      process_data(p);
    }

    void stop() {
      // gpsmm doesn't have a close method? OK ...
    }

  private:
    ros::NodeHandle node;
    ros::NodeHandle privnode;
    ros::Publisher gps_fix_pub;
    ros::Publisher navsat_fix_pub;
    gpsmm *gps;

    bool use_gps_time;
    bool check_fix_by_variance;

    void process_data(struct gps_data_t* p) {
      if (p == NULL) {
        ROS_WARN("GPS data is NULL!");
        return;
      }

      if (!p->online) {
        ROS_WARN("GPS is not online");
        return;
      }

      // ROS_INFO("process_data_gps");
      process_data_gps(p);
      // ROS_INFO("process_data_navsat");
      process_data_navsat(p);
    }


#if GPSD_API_MAJOR_VERSION >= 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#else
#error "gpsd_client only supports gpsd API versions 3+"
#endif

    void process_data_gps(struct gps_data_t* p) {
      ros::Time time = ros::Time::now();

      GPSFix fix;
      GPSStatus status;

      status.header.stamp = time;
      fix.header.stamp = time;

      status.satellites_used = p->satellites_used;

      status.satellite_used_prn.resize(status.satellites_used);
      for (int i = 0; i < status.satellites_used; ++i) {
#if GPSD_API_MAJOR_VERSION > 5
        status.satellite_used_prn[i] = p->skyview[i].used;
#else
        status.satellite_used_prn[i] = p->used[i];
#endif
      }

      status.satellites_visible = SATS_VISIBLE;

      status.satellite_visible_prn.resize(status.satellites_visible);
      status.satellite_visible_z.resize(status.satellites_visible);
      status.satellite_visible_azimuth.resize(status.satellites_visible);
      status.satellite_visible_snr.resize(status.satellites_visible);

      for (int i = 0; i < SATS_VISIBLE; ++i) {
#if GPSD_API_MAJOR_VERSION > 5
        status.satellite_visible_prn[i] = p->skyview[i].PRN;
        status.satellite_visible_z[i] = p->skyview[i].elevation;
        status.satellite_visible_azimuth[i] = p->skyview[i].azimuth;
        status.satellite_visible_snr[i] = p->skyview[i].ss;
#else
        status.satellite_visible_prn[i] = p->PRN[i];
        status.satellite_visible_z[i] = p->elevation[i];
        status.satellite_visible_azimuth[i] = p->azimuth[i];
        status.satellite_visible_snr[i] = p->ss[i];
#endif
      }
      // ROS_INFO("%d", STATUS_FIX);
      if ((p->status & STATUS_FIX) && !(check_fix_by_variance && isnan(p->fix.epx))) {
        status.status = 0; // FIXME: gpsmm puts its constants in the global
                           // namespace, so `GPSStatus::STATUS_FIX' is illegal.

// STATUS_DGPS_FIX was removed in API version 6 but re-added afterward
#if GPSD_API_MAJOR_VERSION != 6
        if (p->status & STATUS_DGPS_FIX)
          status.status |= 18; // same here
#endif

        fix.time = p->fix.time;
        fix.latitude = p->fix.latitude;
        fix.longitude = p->fix.longitude;
        fix.altitude = p->fix.altitude;
        fix.track = p->fix.track;
        fix.speed = p->fix.speed;
        fix.climb = p->fix.climb;

#if GPSD_API_MAJOR_VERSION > 3
        fix.pdop = p->dop.pdop;
        fix.hdop = p->dop.hdop;
        fix.vdop = p->dop.vdop;
        fix.tdop = p->dop.tdop;
        fix.gdop = p->dop.gdop;
#else
        fix.pdop = p->pdop;
        fix.hdop = p->hdop;
        fix.vdop = p->vdop;
        fix.tdop = p->tdop;
        fix.gdop = p->gdop;
#endif

        fix.err = p->epe;
        fix.err_vert = p->fix.epv;
        fix.err_track = p->fix.epd;
        fix.err_speed = p->fix.eps;
        fix.err_climb = p->fix.epc;
        fix.err_time = p->fix.ept;

        /* TODO: attitude */
      } else {
      	status.status = -1; // STATUS_NO_FIX
      }

      fix.status = status;

      gps_fix_pub.publish(fix);
    }

    void process_data_navsat(struct gps_data_t* p) {
      NavSatFixPtr fix(new NavSatFix);
      // ROS_INFO("process navsat");
      /* TODO: Support SBAS and other GBAS. */

      if (use_gps_time && !isnan(p->fix.time)) {
        fix->header.stamp = ros::Time(p->fix.time);
        // ROS_INFO("use_gps_time && !isnan(p->fix.time");
      }
      else {
        // ROS_INFO("fix->header.stamp");
        fix->header.stamp = ros::Time::now();
      }

      /* gpsmm pollutes the global namespace with STATUS_,
       * so we need to use the ROS message's integer values
       * for status.status
       */
      std::string fix_str;                //added by cheng 16/8--------------------------------------------------
      switch (p->status) {
        case STATUS_NO_FIX:
          fix->status.status = -1; // NavSatStatus::STATUS_NO_FIX;
          fix_str = "GPS can't get a fix";  //added by cheng 16/8------------------------------------------------
          break;
        case STATUS_FIX:
          fix->status.status = 0; // NavSatStatus::STATUS_FIX;
          fix_str = "GPS get a normal fix"; //added by cheng 16/8------------------------------------------------
          break;
// STATUS_DGPS_FIX was removed in API version 6 but re-added afterward
#if GPSD_API_MAJOR_VERSION != 6 
        case STATUS_DGPS_FIX:
          fix->status.status = 2; // NavSatStatus::STATUS_GBAS_FIX;
          fix_str = "DGPS fix";             //added by cheng 16/8------------------------------------------------
          break;
#endif
      }
      // ROS_INFO("fix->status.status: %d", fix->status.status);

      fix->status.service = NavSatStatus::SERVICE_GPS;

      fix->latitude = p->fix.latitude;
      fix->longitude = p->fix.longitude;
      fix->altitude = p->fix.altitude;

      /* gpsd reports status=OK even when there is no current fix,
       * as long as there has been a fix previously. Throw out these
       * fake results, which have NaN variance
       */

      /* do not publish msg if there is no fix
       */

      // if (isnan(p->fix.epx) && check_fix_by_variance) {
      //   return;
      // }

//---------added by cheng16/8-----------------------------------------------------------------------------------
      /* publish msg even if there is no fix */

      if (isnan(p->fix.epx) && check_fix_by_variance) {
        fix->status.status = -1;
        fix_str = "GPS can't get a fix";
      }

      ROS_INFO("%s", fix_str.c_str());             

//---------------------------------------------------------------------------------------------------------------

      fix->position_covariance[0] = p->fix.epx;
      fix->position_covariance[4] = p->fix.epy;
      fix->position_covariance[8] = p->fix.epv;

      fix->position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      navsat_fix_pub.publish(fix);
    }
};

//-------added by cheng16/8----------------------------------------
void killall() {
  system("sudo killall gpsd");
}
//-----------------------------------------------------------------

int main(int argc, char ** argv) {
  ros::init(argc, argv, "gpsd_client");

  GPSDClient client;

//-------added by cheng16/8----------------------------------------
  client.run_gpsd();
//-----------------------------------------------------------------

  if (!client.start()) {
    killall();                  //added by cheng16/8---------------
    return -1;
  }


  while(ros::ok()) {
    // ROS_INFO("1");
    ros::spinOnce();
    // ROS_INFO("2");
    client.step();
    // ROS_INFO("3");
  }  


  client.stop();

//-------added by cheng16/8----------------------------------------
  killall();
//-----------------------------------------------------------------
  
}
