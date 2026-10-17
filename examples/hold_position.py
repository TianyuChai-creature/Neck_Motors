"""Hold yaw=0, pitch=35 until released; use the installed wheel and an unlock button."""
import csv
from dataclasses import asdict
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import termios
import time

import f32c
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication,QLabel,QPushButton,QVBoxLayout,QWidget
from capture_pitch_feedback import query

ROOT=Path(__file__).resolve().parent.parent


def run(device, directory, stop_after=None):
    directory=Path(directory)
    app=QApplication.instance() or QApplication([])
    cfg=f32c.Config(device=device,travel_rpm=5,accel=1,
                   yaw_speed_kp=40,yaw_speed_ki=4,yaw_pos_kp=42,yaw_pos_ki=0,
                   pitch_speed_kp=10,pitch_speed_ki=4,pitch_pos_kp=40,pitch_pos_ki=0)
    result={'action':'hold_fixed_position','config':asdict(cfg),'status':'opening',
            'hardware_state':'unknown','module':f32c.__file__}
    motor=None;fd=None;closed=False;last_report=0
    raw=(directory/'telemetry_raw.jsonl').open('x')
    samples=(directory/'samples.csv').open('x',newline='')
    writer=csv.writer(samples);writer.writerow(['monotonic_s','yaw_deg','pitch_deg','yaw_voltage_V','pitch_voltage_V'])
    timer=QTimer();label=QLabel('正在准备固定位置保持…');button=QPushButton('解锁并退出')
    def save():
        temp=directory/'state.json.tmp';temp.write_text(json.dumps(result,indent=2)+'\n');temp.replace(directory/'state.json')
    def release(reason):
        nonlocal closed
        if closed:return
        closed=True;timer.stop();result['status']=reason
        if motor is not None:
            try:motor.disable();result['disable_requests_sent']=True
            except Exception as exc:result['disable_error']=str(exc)
            try:motor.close();result['software_close_completed']=True
            except Exception as exc:result['close_error']=str(exc)
        if fd is not None:
            try:fcntl.ioctl(fd,termios.TIOCNXCL)
            except OSError:pass
            os.close(fd)
        raw.close();samples.close();save();print(json.dumps(result),flush=True)
    class Window(QWidget):
        def closeEvent(self,event):
            release('released');event.accept()
    window=Window();window.setWindowTitle('固定位置保持 · yaw 0° / pitch 35°')
    layout=QVBoxLayout(window);layout.addWidget(label);layout.addWidget(button)
    button.clicked.connect(window.close);window.resize(430,180)
    def volts():
        values=[query(fd,4,raw,address=a)[0]/100 for a in (1,2)]
        if any(not 8<=v<=15 for v in values):raise RuntimeError(f'电压超出8–15V：{values}')
        return values
    def observe():
        nonlocal last_report
        try:
            state=motor.read(timeout_s=.04);v=volts();now=time.monotonic()
            y=(state.yaw_deg+180)%360-180;p=(state.pitch_deg+180)%360-180
            result['measured_deg']={'yaw':y,'pitch':p};result['voltage_V']={'yaw':v[0],'pitch':v[1]}
            result['updated_monotonic_s']=now
            writer.writerow([now,y,p,*v]);samples.flush()
            label.setText(f"保持目标：yaw {target[0]:.1f}° / pitch {target[1]:.1f}°\n"
                          f"当前回读：yaw {y:.1f}° / pitch {p:.1f}°\n"
                          f"电压：yaw {v[0]:.2f} V / pitch {v[1]:.2f} V\n关闭窗口或点击下方按钮会发送失能请求。")
            save()
            if now-last_report>=5:print(json.dumps(result),flush=True);last_report=now
        except Exception as exc:
            result['error']=str(exc);release('fault');label.setText('保持已中止：'+str(exc));button.setText('关闭窗口')
    try:
        assert Path(f32c.__file__).resolve().is_relative_to(Path(sys.prefix).resolve())
        motor=f32c.Gimbal2Dof(cfg)
        assert Path(motor._lib._name).resolve()==(f32c.native_prefix()/'lib/libf32c.so').resolve()
        motor.open();motor.start_log(directory/'serial.log')
        fd=os.open(device,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
        fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB);fcntl.ioctl(fd,termios.TIOCEXCL)
        volts()
        state=motor.read(timeout_s=.04)
        result['initial_deg']={'yaw':(state.yaw_deg+180)%360-180,'pitch':(state.pitch_deg+180)%360-180}
        target=(f32c.DEFAULT_HOLD_POSITION.yaw_deg,f32c.DEFAULT_HOLD_POSITION.pitch_deg)
        result['target_deg']={'yaw':target[0],'pitch':target[1]};save()
        motor.enable_at(*target)
        result['status']='holding';save();observe()
        window.show()
        if not closed:timer.timeout.connect(observe);timer.start(100)
        if stop_after is not None:QTimer.singleShot(round(stop_after*1000),window.close)
        signal.signal(signal.SIGTERM,lambda *_:window.close())
        signal.signal(signal.SIGINT,lambda *_:window.close())
        app.exec()
    except BaseException as exc:
        result['error']=str(exc);release('fault')
    finally:
        if not closed:release('released')
    return result


if __name__=='__main__':
    (ROOT/'logs').mkdir(exist_ok=True)
    output=Path(tempfile.mkdtemp(prefix=datetime.now().strftime('hold_fixed_%Y%m%d_%H%M%S_'),dir=ROOT/'logs'))
    print(output,flush=True)
    run(sys.argv[1] if len(sys.argv)>1 else '/dev/ttyACM0',output)
