string robot_id #로봇ID
string model_id 
string map_id   #장소ID

Location local_position # 현재위치
Velocity velocity # 현재속도
float32 battery

Location local_destination #목적지 좌표
string poi_id #목적지 PoI ID
Location[] local_path #로봇경로

ServiceMode service
TMSMode tms
NavMode nav #네비게이션 모드

