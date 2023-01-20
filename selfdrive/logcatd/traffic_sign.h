#pragma once

/*
MAPPY
    signtype
    111 오른쪽 급커브
    112 왼쪽 급커브
    113 굽은도로
    118, 127 어린이보호구역
    122 : 좁아지는 도로
    124 : 과속방지턱
    129 : 주정차
    131 : 단속카메라(신호위반카메라)  
    135 : 고정식(버스단속구간)  - 호야
    150 : 경찰차(이동식단속구간)  - 호야
    165 : 구간단속    
    198 차선변경금지시작
    199 차선변경금지종료
    129 주정차금지구간
    123 철길건널목
    200 : 단속구간(고정형 이동식)
    231 : 단속(카메라, 신호위반)    
    246 버스전용차로단속
    247 과적단속
    248 교통정보수집
    249 추월금지구간
    250 갓길단속
    251 적재불량단속
*/


typedef enum TrafficSign {
  TS_CURVE_RIGHT = 111,  // 오른쪽 급커브
  TS_CURVE_LEFT = 112,   // 왼쪽 급커브
  TS_BEND_ROAD = 113,    // 굽은도로
  TS_SCHOOL_ZONE1 = 118,  // 어린이보호구역
  TS_SCHOOL_ZONE2 = 127,  // 어린이보호구역
  TS_NARROW_ROAD = 122,   // 좁아지는 도로
  TS_RAIL_ROAD = 123,     // 철길건널목
  TS_BUMP_ROAD =  124,  // 과속방지턱
  TS_PARK_CRACKDOWN  = 129,  // 주정차단속
  TS_CAMERA1  = 131,  // 단속카메라(신호위반카메라)  
  TS_CAMERA2_BUS  = 135,  // 고정식  - 호야
  TS_CAMERA3  = 150,  // 경찰차(이동식)  - 호야
  TS_INTERVAL  = 165,  // 구간 단속
  TS_VARIABLE  = 195,  // 가변구간
  TS_LANE_CHANGE1  = 198,  // 차선변경금지시작
  TS_ANE_CHANGE2  = 199,  // 차선변경금지종료
  TS_PARK_ZONE  = 129,  // 주정차금지구간
  TS_RAILROAD  = 123,  // 철길건널목
  TS_CAMERA4  = 200,  // 단속구간(고정형 이동식)
  TS_CAMERA5  = 231,  // 단속(카메라, 신호위반)    
  TS_BUS_ONLY  = 246,  // 버스전용차로단속
  TS_LOAD_OVER  = 247,  // 과적단속
  TS_TRAFFIC_INFO  = 248,  // 교통정보수집
  TS_OVERTRAK  = 249,  // 추월금지구간
  TS_SHOULDER  = 250,  // 갓길단속
  TS_LOAD_POOR  = 251,  // 적재불량단속  
} TrafficSign;
