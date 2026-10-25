"""Compile the actual RC, shooter and motor adapter with minimal hardware stubs."""
from pathlib import Path
import os
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[1]
src = repo / 'RCS_code' / 'STM32F405'


def read(name):
    return (src / name).read_bytes().decode('latin1').replace('\r\n', '\n')


def method(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 0
    for end in range(brace, len(text)):
        depth += (text[end] == '{') - (text[end] == '}')
        if depth == 0:
            return text[start:end + 1]
    raise AssertionError(signature)


rc = read('RC.cpp')
start = rc.index('\t\t\tfire_now = (rc.ch[1]', rc.index('case CONTROL::FIRE:'))
arbitrate = rc[start:rc.index('\n\t\t}', start)]
harness = r'''
#include "feeder.h"
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
enum {speed=0, INTEGRATE=0, ID1=0x201, XUC_CONTROL_AUTOAIM=1};
uint32_t clock_ms=2;
uint32_t HAL_GetTick(){return clock_ms;}
struct FakeCAN {
 uint8_t frame[8]{}; bool available=true;
 bool MotorFeedback(uint32_t, uint8_t out[8]){std::memcpy(out,frame,8);return available;}
} can2;
struct PID {float m_error[1]{};float Position(float e,float cap){return std::max(-cap,std::min(cap,20*e));}};
struct Motor {
 enum {stop=0, single=2};
 uint32_t ID=ID1+2,feeder_last_start=0,shoot_num=0;
 FeederController feeder;
 bool single_requested=false;
 int setspeed=0,setcurrent=0,current=0,shoot_mode_now=stop;
 float distance=0;
 PID pid[1];
 static int16_t getword(uint8_t high,uint8_t low){return static_cast<int16_t>((high<<8)|low);}
 static int32_t setrange(int32_t value,int32_t range){return std::max(-range,std::min(range,value));}
 void UpdateFeeder();
} can2_motor[3],can1_motor[4];
struct CONTROL {
 enum {RESET,FIRE,AUTOAIM,REAL};int mode=RESET;
 struct SHOOTER {
  bool return_started=false,feeder_real_counting=false,fire_now_single=false,fire_last_single=false;
  uint32_t feeder_real_baseline=0;
  void Update();
 } shooter;
 struct {bool real_returning=false;float real_d0=0,real_d1=0,real_d2=0,real_d3=0,real_yaw_target=0;} chassis;
} ctrl;
struct XUC {
 struct {int mode_TJ=0;} Tx_TJ;
 struct {int shoot_TJ=0,control_TJ=XUC_CONTROL_AUTOAIM;} Rx_TJ;
 bool fresh=true;bool RxFresh(){return fresh;}
} xuc;
struct {float GetAngleYaw(){return 123;}} imu_pantile;
struct RC {struct {int ch[4]{};} rc;bool fire_now=false,fire_last=false;void Step();};
#define CHECK(x) do {if(!(x)){std::printf("FAIL %d: %s\n",__LINE__,#x);std::exit(1);}} while(0)
void reset(){
 ctrl=CONTROL();xuc=XUC();can2=FakeCAN();
 for(int i=0;i<3;++i)can2_motor[i]=Motor();
}
'''
harness += method(read('motor.cpp'), 'void Motor::UpdateFeeder()') + '\n'
harness += method(read('control.cpp'), 'void CONTROL::SHOOTER::Update()') + '\n'
harness += 'void RC::Step(){\n' + arbitrate + '\n}\n'
harness += r'''
int main(){
 { // Continuous FIRE channel alone cannot request or move the feeder.
  reset();ctrl.mode=CONTROL::FIRE;ctrl.shooter.Update();RC r;
  const int continuous_values[]={300,-300,0};
  for(int value:continuous_values){r.rc.ch[0]=value;r.Step();can2_motor[2].UpdateFeeder();
   CHECK(!can2_motor[2].single_requested && can2_motor[2].feeder.started_count==0);
   CHECK(can2_motor[2].setcurrent==0);}
  r.rc.ch[0]=300;r.rc.ch[1]=300;r.Step();can2_motor[2].UpdateFeeder();
  CHECK(can2_motor[2].feeder.started_count==1 && can2_motor[2].feeder.target==42130);
  CHECK(can2_motor[2].setspeed==150 && can2_motor[2].setcurrent>0);
  r.Step();CHECK(!can2_motor[2].single_requested);
  r.rc.ch[1]=0;r.Step();r.rc.ch[1]=-300;r.Step();
  CHECK(!can2_motor[2].single_requested); // Busy edges are discarded.
  can2_motor[2].feeder.state=FeederController::IDLE;can2_motor[2].UpdateFeeder();
  r.Step();CHECK(!can2_motor[2].single_requested); // Held input is not replayed.
  r.rc.ch[1]=0;r.Step();r.rc.ch[1]=300;r.Step();
  CHECK(can2_motor[2].single_requested);
 }
 { // A request with friction-wheel commands off must produce no current.
  reset();can2_motor[2].single_requested=true;can2_motor[2].UpdateFeeder();
  CHECK(can2_motor[2].feeder.started_count==0 && can2_motor[2].setcurrent==0);
 }
 { // AUTOAIM entry does not move; only a valid command edge starts a shot.
  reset();ctrl.mode=CONTROL::AUTOAIM;ctrl.shooter.Update();can2_motor[2].UpdateFeeder();
  CHECK(can2_motor[2].feeder.started_count==0);
  xuc.fresh=false;xuc.Rx_TJ.shoot_TJ=1;ctrl.shooter.Update();
  CHECK(!can2_motor[2].single_requested);
  xuc.fresh=true;ctrl.shooter.Update();can2_motor[2].UpdateFeeder();
  CHECK(can2_motor[2].feeder.started_count==1);
  ctrl.shooter.Update();CHECK(!can2_motor[2].single_requested);
 }
 { // REAL travel suppresses shooting; count accepted starts and finish shot 50.
  reset();ctrl.mode=CONTROL::REAL;can2_motor[2].feeder.started_count=40;
  xuc.Rx_TJ.shoot_TJ=1;ctrl.shooter.Update();can2_motor[2].UpdateFeeder();
  CHECK(can2_motor[2].shoot_num==0 && can2_motor[2].setspeed==0);
  CHECK(!can2_motor[2].single_requested && can2_motor[0].setspeed==0);
  xuc.Tx_TJ.mode_TJ=1;can2_motor[2].feeder.started_count=89;
  ctrl.shooter.Update();CHECK(can2_motor[2].shoot_num==49);
  CHECK(can2_motor[2].single_requested);
  can2_motor[2].UpdateFeeder();CHECK(can2_motor[2].feeder.started_count==90);
  ctrl.shooter.Update();CHECK(can2_motor[2].shoot_num==50 && !ctrl.chassis.real_returning);
  xuc.Rx_TJ.shoot_TJ=0;ctrl.shooter.Update();xuc.Rx_TJ.shoot_TJ=1;ctrl.shooter.Update();
  CHECK(!can2_motor[2].single_requested); // Shot 51 is blocked.
  can2_motor[2].feeder.state=FeederController::SETTLE;ctrl.shooter.Update();
  CHECK(!ctrl.chassis.real_returning);
  can2_motor[2].feeder.state=FeederController::IDLE;ctrl.shooter.Update();
  CHECK(ctrl.chassis.real_returning && xuc.Tx_TJ.mode_TJ==0);
 }
 { // No former feeder trip condition can latch the mechanism in any firing mode.
  const int modes[]={CONTROL::FIRE,CONTROL::AUTOAIM,CONTROL::REAL};
  for(int mode:modes){
   reset();ctrl.mode=mode;xuc.Tx_TJ.mode_TJ=1;ctrl.shooter.Update();
   can2.frame[6]=80;can2_motor[2].single_requested=true;can2_motor[2].UpdateFeeder();
   CHECK(can2_motor[2].feeder.state==FeederController::RUN && can2_motor[2].setcurrent>0);
   for(int i=0;i<1200;++i){clock_ms+=2;can2_motor[2].UpdateFeeder();}
   CHECK(can2_motor[2].feeder.state==FeederController::RUN && can2_motor[2].setcurrent>0);
   can2.available=false;can2_motor[2].UpdateFeeder();ctrl.shooter.Update();
   CHECK(can2_motor[0].setspeed==-7000 && can2_motor[1].setspeed==7000);
   CHECK(can2_motor[2].feeder.state==FeederController::RUN);
   can2.available=true;
   ctrl.mode=CONTROL::RESET;ctrl.shooter.Update();
   CHECK(can2_motor[0].setspeed==0 && can2_motor[1].setspeed==0);
   can2_motor[2].UpdateFeeder();CHECK(can2_motor[2].setcurrent==0);
   ctrl.mode=mode;ctrl.shooter.Update();
   if(mode==CONTROL::FIRE)can2_motor[2].single_requested=true;
   else{xuc.Rx_TJ.shoot_TJ=1;ctrl.shooter.Update();}
   can2_motor[2].UpdateFeeder();
   CHECK(can2_motor[2].feeder.state==FeederController::RUN && can2_motor[2].feeder.started_count==2);
  }
  reset();ctrl.mode=CONTROL::REAL;
  ctrl.shooter.Update();CHECK(can2_motor[0].setspeed==0 && can2_motor[1].setspeed==0);
  xuc.Tx_TJ.mode_TJ=1;ctrl.chassis.real_returning=true;ctrl.shooter.Update();
  CHECK(can2_motor[0].setspeed==0 && can2_motor[1].setspeed==0);
 }
 puts("PASS: actual RC/SHOOTER/Motor adapter: continuous disabled, single edges, wheel gate, AUTOAIM, REAL completion, no feeder fault latch");
}
'''
with tempfile.TemporaryDirectory(prefix='omni-feeder-control-') as work:
    p = Path(work)
    (p / 'control_test.cpp').write_bytes(harness.encode('latin1'))
    env = os.environ.copy()
    env['PATH'] = r'C:\msys64\ucrt64\bin;' + env['PATH']
    subprocess.run([r'C:\msys64\ucrt64\bin\g++.exe', '-std=c++11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(src), str(p / 'control_test.cpp'), '-o', str(p / 'control_test.exe')],
                   env=env, check=True)
    subprocess.run([str(p / 'control_test.exe')], env=env, check=True)
