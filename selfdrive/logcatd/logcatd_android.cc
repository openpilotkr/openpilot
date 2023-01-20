#include <sys/time.h>
#include <sys/resource.h>

#include <android/log.h>
#include <log/logger.h>
#include <log/logprint.h>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/util.h"
#include "selfdrive/common/params.h"


// atom
typedef struct LiveNaviDataResult {
      int speedLimit = 0;  // int;
      float safetyDistance = 0;  // Float32;
      int safetySign = 0;    // int;
      int safetySignCam = 0;    // int;
      float roadCurvature = 0;    // Float32;
      int turnInfo = 0;    // int;
      float distanceToTurn = 0;    // Float32;
      //bool  mapValid;    // bool;
      //int  mapEnable;    // bool;
      long  tv_sec;
      long  tv_nsec;
} LiveNaviDataResult;


int main() {
  setpriority(PRIO_PROCESS, 0, -15);
  long  nDelta_nsec = 0;
  long  tv_nsec;
  float tv_nsec2;
  bool  sBump = false;
  int   naviSel = std::stoi(Params().get("OPKRNaviSelect"));

  ExitHandler do_exit;
  PubMaster pm({"liveNaviData"});
  LiveNaviDataResult res;

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
   // struct logger *radio_logger = android_logger_open(logger_list, LOG_ID_RADIO);
   // assert(radio_logger);
   // struct logger *system_logger = android_logger_open(logger_list, LOG_ID_SYSTEM);
   // assert(system_logger);
   // struct logger *crash_logger = android_logger_open(logger_list, LOG_ID_CRASH);
   // assert(crash_logger);
   // struct logger *kernel_logger = android_logger_open(logger_list, (log_id_t)5); // LOG_ID_KERNEL
   // assert(kernel_logger);

    while (!do_exit) {
      log_msg log_msg;
      int err = android_logger_list_read(logger_list, &log_msg);
      if (err <= 0) break;

      AndroidLogEntry entry;
      err = android_log_processLogBuffer(&log_msg.entry_v1, &entry);
      if (err < 0) continue;
      last_log_time.tv_sec = entry.tv_sec;
      last_log_time.tv_nsec = entry.tv_nsec;

      tv_nsec2 = entry.tv_nsec / 1000000;
      tv_nsec =  entry.tv_sec * 1000ULL + long(tv_nsec2); // per 1000 = 1s

      MessageBuilder msg;
      auto framed = msg.initEvent().initLiveNaviData();

   //  opkrspdlimit, opkrspddist, opkrsigntype, opkrcurveangle, opkrremaintime, opkradasicontype, opkraveragespd

      // code based from atom
      nDelta_nsec = tv_nsec - res.tv_nsec;
      //nDelta = entry.tv_sec - res.tv_sec;

      if( strcmp( entry.tag, "opkrspddist" ) == 0 )
      {
        res.tv_sec = entry.tv_sec;
        res.tv_nsec = tv_nsec;
        res.safetyDistance = atoi( entry.message );
      }
      else if( strcmp( entry.tag, "opkrspdlimit" ) == 0 )
      {
        res.speedLimit = atoi( entry.message );
      }
      else if( strcmp( entry.tag, "opkrsigntype" ) == 0 )
      {
        res.tv_sec = entry.tv_sec;
        res.tv_nsec = tv_nsec;
        res.safetySignCam = atoi( entry.message );
        if (res.safetySignCam == 124 && naviSel == 1) {
          sBump = true;
        }
      }
      else if( strcmp( entry.tag, "opkrroadsigntype" ) == 0 )
      {
        res.safetySign = atoi( entry.message );
      }
      else if( naviSel == 1 && (res.safetyDistance > 1 && res.safetyDistance < 60) && (strcmp( entry.tag, "AudioFlinger" ) == 0) )  //   msm8974_platform
      {
        res.safetyDistance = 0;
        res.speedLimit = 0;
        res.safetySignCam = 0;
      }
      else if( strcmp( entry.tag, "opkrturninfo" ) == 0 )
      {
        res.turnInfo = atoi( entry.message );
      }
      else if( strcmp( entry.tag, "opkrdistancetoturn" ) == 0 )
      {
        res.distanceToTurn = atoi( entry.message );
      }
      else if( nDelta_nsec > 10000 && naviSel == 0)
      {
        res.tv_sec = entry.tv_sec;
        res.tv_nsec = tv_nsec;
        res.safetyDistance = 0;
        res.speedLimit = 0;
        res.safetySign = 0;
        // system("logcat -c &");
      }
      else if( nDelta_nsec > 5000 && naviSel == 1)
      {
        if (res.safetySignCam == 197 && res.safetyDistance < 100) {
          res.safetyDistance = 0;
          res.speedLimit = 0;
          res.safetySignCam = 0;
        }
        else if ( res.safetySignCam == 124 && (!sBump) )
        {
          res.safetySignCam = 0;
        }
        else if (res.safetySignCam != 0 && res.safetySignCam != 124 && res.safetyDistance < 50 && res.safetyDistance > 0)
        {
          res.safetyDistance = 0;
          res.speedLimit = 0;
          res.safetySignCam = 0;
        }
        else if( nDelta_nsec > 10000 )
        {
          res.tv_sec = entry.tv_sec;
          res.tv_nsec = tv_nsec;
          res.safetyDistance = 0;
          res.speedLimit = 0;
          res.safetySignCam = 0;
          // system("logcat -c &");
        }
      }

      framed.setSpeedLimit( res.speedLimit );  // int;
      framed.setSafetyDistance( res.safetyDistance );  // raw_target_speed_map_dist Float32;
      framed.setSafetySign( res.safetySign ); // int;
      framed.setSafetySignCam( res.safetySignCam ); // int;
      // framed.setRoadCurvature( res.roadCurvature ); // road_curvature Float32;
      framed.setTurnInfo( res.turnInfo );  // int;
      framed.setDistanceToTurn( res.distanceToTurn );  // Float32;
      framed.setTs( res.tv_sec );
      //framed.setMapEnable( res.mapEnable );
      //framed.setMapValid( res.mapValid );

    /*
    iNavi Road signtype
    101 연속 커브
    102 추돌주의
    107 과속방지턱
    5 이동식
    105 낙석주의
    15 고정식
    10 합류
    9 과적단속
    111 철길건널목
    18 이벤트 발생
    203 녹색교통

    iNavi Cam signtype
    11, 12 구간단속
    6 이동식단속
    2 신호및속도단속
    1 안전속도
    3 신호위반단속
    4, 7 버스전용차로 단속
    5 교통량 측정
    8 주차위반 단속
    101 연속 커브
    15 박스형카메라
    16 스쿨존
    18 실버존
    118 야생동몰
    20 차선변경금지
    203 녹색교통
    204 미끄럼주의


    mappy signtype
    111 오른쪽 급커브
    112 왼쪽 급커브
    113 굽은도로
    118, 127 어린이보호구역
    122 좁아지는 도로
    124 과속방지턱
    129 주정차
    131 단속카메라(신호위반카메라)  
    135 고정식(버스단속구간)  - 호야
    150 경찰차(이동식단속구간)  - 호야
    165 구간단속
    195, 197 구간단속, 가변속도제한구간
    198 차선변경금지시작
    199 차선변경금지종료
    129 주정차금지구간
    123 철길건널목
    200 단속구간(고정형 이동식)
    231 단속(카메라, 신호위반)
    246 버스전용차로단속
    247 과적단속
    248 교통정보수집
    249 추월금지구간
    250 갓길단속
    251 적재불량단속
   */  


      pm.send("liveNaviData", msg);
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
