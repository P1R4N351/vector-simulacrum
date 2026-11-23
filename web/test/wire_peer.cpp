// Integration peer built with the unmodified WireOS v6 headers.
#include "vic_bridge_protocol.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdio>
#include <cmath>

int main() {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = htons(5800);
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  timeval timeout{30,0};
  if (fd < 0 || bind(fd,(sockaddr*)&local,sizeof(local)) || setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))) return 1;
  VicBridgeB2H body{};
  int start[4]{};
  uint32_t last = 0;
  for (int i=0;i<=120;++i) {
    sockaddr_in peer{};
    socklen_t len = sizeof(peer);
    if (recvfrom(fd,&body,sizeof(body),0,(sockaddr*)&peer,&len)!=sizeof(body)) return 2;
    if (body.magic!=VIC_BRIDGE_MAGIC || body.version!=VIC_BRIDGE_PROTO_VERSION ||
        body.b2h.framecounter!=body.seq || (i && body.seq<=last) ||
        !(body.b2h.flags & RUNNING_FLAGS_SENSORS_VALID) || !std::isfinite(body.imu.accel[2])) return 3;
    last = body.seq;
    if (!i) for(int m=0;m<4;++m) start[m]=body.b2h.motor[m].position;
    VicBridgeH2B cmd{};
    cmd.magic=body.magic; cmd.version=body.version; cmd.seq=body.seq;
    cmd.h2b.motorPower[MOTOR_LEFT]=3000; cmd.h2b.motorPower[MOTOR_RIGHT]=-3000;
    cmd.h2b.motorPower[MOTOR_HEAD]=3500; cmd.h2b.motorPower[MOTOR_LIFT]=1000;
    if (sendto(fd,&cmd,sizeof(cmd),0,(sockaddr*)&peer,len)!=sizeof(cmd)) return 4;
  }
  close(fd);
  int change[4];
  for(int m=0;m<4;++m) change[m]=body.b2h.motor[m].position-start[m];
  std::printf("121 frames, encoder changes L/R/lift/head %d %d %d %d; accel Z %.1f mm/s2, battery %.2f V\n",
    change[0],change[1],change[2],change[3],body.imu.accel[2],body.b2h.battery.main_voltage*2.8/2048);
  return !(change[0]>0 && change[1]<0 && change[2]>0 && change[3]>0);
}
