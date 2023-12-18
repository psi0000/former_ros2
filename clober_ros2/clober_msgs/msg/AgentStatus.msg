string robot_id # 로봇 ID
string robot_name
string place_id # 장소 ID
string model_id
string map_id # 장소 ID

bool is_enabled # 연결 여부

bool connection # 연결 여부
Verification verification # 검증 확인 여부

Location position # 현재 위치(x, y, yaw)
Location local_position # 현재 위치(x, y, yaw)
Velocity velocity # 현재 속도(x, y, yaw)
float32 battery

ServiceMode service
TMSMode tms
NavMode nav

Location destination # 목적지 좌표(x, y, yaw)
Location local_destination
string poi_id # 목적지 PoI ID
uint8 goal_type

Location[] path # 로봇 경로
Location[] local_path

NavMode nav_mode # 네비게이션 모드

bool is_manual
bool is_emergency
string halt_status