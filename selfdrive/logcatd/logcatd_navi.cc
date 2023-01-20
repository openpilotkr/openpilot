#include <sys/time.h>
#include <sys/resource.h>

#include <android/log.h>
#include <log/logger.h>
#include <log/logprint.h>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/util.h"
#include "selfdrive/common/params.h"

#include "selfdrive/logcatd/traffic_sign.h"

typedef struct LiveNaviDataResult {
      int   speedLimit;  // int;
      float safetyDistance;  // Float32;
      int   safetySign;    // int;
      float roadCurvature;    // Float32;
      int   turnInfo;    // Int32;
      float distanceToTurn;    // Float32;      
      bool  mapValid;    // bool;
      int   mapEnable;    // Int32;

      double  dArrivalDistance;    // unit:  M
      double  dArrivalTimeSec;    // unit: sec
      double  dEventSec;
      double  dHideTimeSec;

      long  tv_sec;
} LiveNaviDataResult;




int traffic_camera( int nsignal_type, float fDistance )
{
    int ret_code = 0;

    switch( nsignal_type )
    {
      case  TS_CAMERA1:  // 단속(카메라, 신호위반) 
      case  TS_CAMERA2_BUS:
      case  TS_CAMERA3:
      case  TS_CAMERA4:  // 단속구간(고정형 이동식)
      case  TS_CAMERA5:  // 단속(카메라, 신호위반)
      case  TS_TRAFFIC_INFO:  // 교통정보수집
        ret_code = 1;
        break;

      case  TS_INTERVAL:  // 구간단속
        if(fDistance < 800)
            ret_code = 1;
        break;
    } 

    return ret_code;
}

// return sec
float arrival_time( float fDistance, float fSpeed_ms )
{
   float  farrivalTime = 0.0;

   if( fSpeed_ms )
    farrivalTime = fDistance / fSpeed_ms;
  else
    farrivalTime = fDistance;
   return farrivalTime;
}


void update_event(  LiveNaviDataResult *pEvet, float  dSpeed_ms )
{
    float  dEventDistance = pEvet->safetyDistance;
    float  dArrivalSec;

    if( dEventDistance > 10 ) {}
    else if(  pEvet->safetySign == 124 ) // 과속방지턱
    {
        dEventDistance = 200;
    }

    if( dEventDistance > 10 )
    {
      dArrivalSec = arrival_time( dEventDistance, dSpeed_ms );

      pEvet->dHideTimeSec = pEvet->dEventSec + dArrivalSec;

      pEvet->dArrivalTimeSec =  dArrivalSec;
      pEvet->dArrivalDistance =  dEventDistance;
    }
    else
    {
      pEvet->dHideTimeSec =  pEvet->dEventSec + 3;
    }
}

int main() {
  setpriority(PRIO_PROCESS, 0, -15);
  long     nLastTime = 0;
  int      traffic_type;
  int      opkr =0;
  double   dArrivalDistanceStop = 0;

  double  dEventLastSec, dEventHideSec;
  double  dCurrentSec;

  ExitHandler do_exit;
  PubMaster pm({"liveNaviData"});
  SubMaster sm({"carState"});
  LiveNaviDataResult  event;

  log_time last_log_time = {};
  logger_list *logger_list = android_logger_list_alloc(ANDROID_LOG_RDONLY | ANDROID_LOG_NONBLOCK, 0, 0);

  while (!do_exit) {
    // setup android logging
    if (!logger_list) {
      logger_list = android_logger_list_alloc_time(ANDROID_LOG_RDONLY | ANDROID_LOG_NONBLOCK, last_log_time, 0);
    }
    assert(logger_list);

    struct logger *main_logger = android_logger_open(logger_list, LOG_ID_MAIN);
    assert(main_logger);


    while (!do_exit) {
    
      sm.update(0);
      const float dSpeed_ms = sm["carState"].getCarState().getVEgo();

      struct timeval t;
      gettimeofday(&t, NULL);
      dCurrentSec = t.tv_sec + 1.0e-9*t.tv_usec;
  
      log_msg log_msg;
      int err = android_logger_list_read(logger_list, &log_msg);
      if (err <= 0) break;

      
      AndroidLogEntry entry;
      err = android_log_processLogBuffer(&log_msg.entry_v1, &entry);
      if (err < 0) continue;
      last_log_time.tv_sec = entry.tv_sec;
      last_log_time.tv_nsec = entry.tv_nsec;

      // break;

      dCurrentSec = entry.tv_sec + 1.0e-9*entry.tv_nsec;
      long nDelta2;
      nDelta2 = entry.tv_sec - nLastTime;
      if( nDelta2 >= 5 )
      {
        nLastTime = entry.tv_sec;
       // event.mapEnable = Params().getInt("OpkrMapEnable");
      }
      
      
      // 2. MAP data Event.
      traffic_type = traffic_camera( event.safetySign, event.safetyDistance );
      if( strcmp( entry.tag, "opkrspddist" ) == 0 )  // 1
      {
        event.safetyDistance = atoi( entry.message );
        opkr = 1;
      } 
      else if( strcmp( entry.tag, "opkrspdlimit" ) == 0 ) // 2
      {
        event.speedLimit = atoi( entry.message );
        opkr = 2;
      }
      else if( strcmp( entry.tag, "opkrcurvangle" ) == 0 )  // 3
      {
        event.roadCurvature = atoi( entry.message );
        opkr = 3;
      }
      else if( strcmp( entry.tag, "opkrsigntype" ) == 0 )  // 4.
      {
        event.safetySign = atoi( entry.message );
        opkr = 4;
        event.dEventSec = dCurrentSec;
        update_event( &event, dSpeed_ms );
      }
      else if( strcmp( entry.tag, "opkrturninfo" ) == 0 )
      {
        event.turnInfo = atoi( entry.message );
      } 
      else if( strcmp( entry.tag, "opkrdistancetoturn" ) == 0 )
      {
        event.distanceToTurn = atoi( entry.message );
      }


      
      
      // 3. Message hide process.
      if( opkr )
      {
        if( dSpeed_ms > 2.0 )
        {
          dEventLastSec = dCurrentSec - event.dEventSec;  // 마지막 Event Time
          event.dArrivalTimeSec = event.dHideTimeSec - dCurrentSec;
          event.dArrivalDistance =  event.dArrivalTimeSec * dSpeed_ms;
          dArrivalDistanceStop = event.dArrivalDistance;

          dEventHideSec = 3;
          if( dSpeed_ms < 10 )
            dEventHideSec = 5;
          else if( dSpeed_ms < 15 )
            dEventHideSec = 4;

          if( dEventLastSec > dEventHideSec )   opkr = 0;
          else if( event.dArrivalTimeSec < 1.5 )  opkr = 0;
        }
        else
        {
          event.dEventSec = dCurrentSec;
          update_event( &event, 2.0 );          
        }       
      }
      else
      {
        event.dHideTimeSec = dCurrentSec + 5;
      }

      if ( opkr )
      {
         event.mapValid = 1;
      }
      else
      {
         event.mapValid = 0; 
         event.safetyDistance = 0;
         event.speedLimit = 0;
         event.safetySign = 0;
      }
           

      /*
      MessageBuilder msg;
      auto framed = msg.initEvent().initLiveNaviData();
      framed.setId(log_msg.id());
      framed.setTs( event.tv_sec );
      framed.setSpeedLimit( event.speedLimit );  // Float32;
      framed.setSafetyDistance( event.safetyDistance );  // raw_target_speed_map_dist Float32;
      framed.setSafetySign( event.safetySign ); // map_sign Float32;
      framed.setRoadCurvature( event.roadCurvature ); // road_curvature Float32;

      // Turn Info
      framed.setTurnInfo( event.turnInfo );
      framed.setDistanceToTurn( event.distanceToTurn );

      framed.setMapEnable( event.mapEnable );
      framed.setMapValid( event.mapValid );
      framed.setTrafficType( traffic_type );

      framed.setArrivalSec(  event.dArrivalTimeSec );
      framed.setArrivalDistance(  event.dArrivalDistance );
*/

      //if( opkr )
      //{
      // printf("logcat ID(%d) - PID=%d tag=%d.[%s] \n", log_msg.id(),  entry.pid,  entry.tid, entry.tag);
      // printf("entry.message=[%s]  \n", entry.message );
      //}
     /*
      pm.send("liveNaviData", msg);
     */
      
      
 
      // OPKR 호환.
      MessageBuilder msg2;
      auto framed2 = msg2.initEvent().initLiveMapData();
      framed2.setSpeedLimit( event.speedLimit );  
      framed2.setSafetyDistance( event.safetyDistance );
      framed2.setSafetySign( event.safetySign );
      framed2.setTurnInfo( event.turnInfo );  
      framed2.setDistanceToTurn( event.distanceToTurn ); 
      framed2.setTs( event.tv_sec );

      pm.send("liveMapData", msg2);

    }

    android_logger_list_free(logger_list);
    logger_list = NULL;
    util::sleep_for(500);
  }

  if (logger_list) {
    android_logger_list_free(logger_list);
  }

  return 0;
}

