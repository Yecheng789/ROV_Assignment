#!/usr/bin/env python3

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from prepare_world import build, ROOT


def camera(parent, name, pose, width, height, fov, fps):
    sensor = ET.SubElement(parent, 'sensor', name=name, type='camera')
    ET.SubElement(sensor, 'pose').text = ' '.join(map(str, pose))
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = str(fps)
    ET.SubElement(sensor, 'topic').text = '/recording/'+name
    settings = ET.SubElement(sensor, 'camera')
    ET.SubElement(settings, 'horizontal_fov').text = str(fov)
    image = ET.SubElement(settings, 'image')
    for tag, value in [('width',width),('height',height),('format','R8G8B8')]:
        ET.SubElement(image,tag).text = str(value)
    clip = ET.SubElement(settings,'clip')
    ET.SubElement(clip,'near').text = '0.05'
    ET.SubElement(clip,'far').text = '100'


class Recorder:
    def __init__(self, name, output, width, height, fps):
        self.name, self.fps = name, fps
        self.width,self.height = width,height
        self.queue = queue.Queue(maxsize=48)
        self.received = self.dropped = self.written = self.repeated = 0
        self.first_time = self.last_time = None
        self.error = None
        self.log = (output/(name+'_ffmpeg.log')).open('w')
        self.timestamps = (output/(name+'_frames.csv')).open('w')
        self.timestamps.write('encoded_frame,source_simulation_time_s,repeated\n')
        self.proc = subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-y',
            '-f','rawvideo','-pixel_format','rgb24','-video_size',f'{width}x{height}',
            '-framerate',str(fps),'-i','pipe:0','-an','-c:v','libx264','-threads','2',
            '-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',
            str(output/(name+'_raw.mp4'))], stdin=subprocess.PIPE,stdout=self.log,stderr=self.log)
        self.thread = threading.Thread(target=self.encode,daemon=True)
        self.thread.start()

    def receive(self, msg):
        if msg.width!=self.width or msg.height!=self.height:
            self.error='Unexpected camera dimensions'; return
        stamp = msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
        self.received += 1
        try:
            self.queue.put_nowait((stamp,bytes(msg.data)))
        except queue.Full:
            self.dropped += 1

    def write(self, pixels, stamp, repeated):
        self.proc.stdin.write(pixels)
        self.timestamps.write(f'{self.written},{stamp:.9f},{int(repeated)}\n')
        self.written += 1
        self.repeated += int(repeated)

    def encode(self):
        previous = None
        try:
            while True:
                item = self.queue.get()
                if item is None: break
                stamp,pixels = item
                if self.first_time is None: self.first_time=stamp
                target = round((stamp-self.first_time)*self.fps)
                if target < self.written: continue
                while self.written < target and previous is not None:
                    self.write(previous,self.last_time,True)
                self.write(pixels,stamp,False)
                self.last_time=stamp; previous=pixels
        except Exception as exc:
            self.error=repr(exc)
        finally:
            self.proc.stdin.close()
            self.proc.wait(timeout=60)
            self.timestamps.close(); self.log.close()

    def close(self):
        if self.thread.is_alive(): self.queue.put(None)
        self.thread.join(timeout=60)
        if self.thread.is_alive() or self.error or self.proc.returncode:
            raise RuntimeError(f'{self.name} encoding failed: {self.error}')
        return {'frames':self.written,'received':self.received,'queue_drops':self.dropped,
                'repeated_frames':self.repeated,'first_sim_time_s':self.first_time,
                'last_sim_time_s':self.last_time,'width':self.width,'height':self.height,
                'fps':self.fps,'duration_s':self.written/self.fps}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fps',type=int,default=15)
    parser.add_argument('--real-time-factor',type=float,default=.6)
    parser.add_argument('--preview-seconds',type=float,default=0)
    args=parser.parse_args()
    out=args.output.resolve()
    if out.exists(): parser.error('Choose a new output directory.')
    out.mkdir(parents=True)
    scenario=json.loads((ROOT/'config/scenario.json').read_text())
    scenario['real_time_factor']=args.real_time_factor
    (out/'scenario.json').write_text(json.dumps(scenario,indent=2)+'\n')
    shutil.copyfile(ROOT/'config/mission.json',out/'mission.json')
    build(scenario,out/'world.sdf')
    tree=ET.parse(out/'world.sdf'); world=tree.getroot().find('world')
    observer=ET.SubElement(world,'model',name='recording_overview')
    ET.SubElement(observer,'static').text='true'
    link=ET.SubElement(observer,'link',name='camera')
    camera(link,'overview',[4,4,10,0,math.pi/2,math.pi/2],1280,1080,.82,args.fps)
    base=world.find("model[@name='bluerov2_ping']/link[@name='base_link']")
    pos=(-1.6,-1.2,1.1); target=(.25,0,-.02)
    delta=[b-a for a,b in zip(pos,target)]
    pitch=-math.atan2(delta[2],math.hypot(*delta[:2])); yaw=math.atan2(delta[1],delta[0])
    camera(base,'chase',[*pos,0,pitch,yaw],640,480,1.1,args.fps)
    ET.indent(tree,'  '); tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
    os.environ.update(GZ_SIM_RESOURCE_PATH=str(ROOT/'vendor/bluerov2_gz/models'),
        GZ_PARTITION='rov_video_'+uuid.uuid4().hex[:10],GZ_HOMEDIR=str(out/'gz_home'),
        ROS_LOG_DIR=str(out/'ros_logs'),ROS_DOMAIN_ID='83',
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        PYTHONPATH=str(ROOT)+os.pathsep+os.environ.get('PYTHONPATH',''),PYTHONUNBUFFERED='1')
    os.environ.pop('ROS_STATIC_PEERS',None)
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    transport=Node()
    recordings=[Recorder('overview',out,1280,1080,args.fps),Recorder('chase',out,640,480,args.fps)]
    for rec in recordings:
        if not transport.subscribe(Image,'/recording/'+rec.name,rec.receive):
            raise RuntimeError('Camera subscription failed.')
    manifest={'command':sys.argv,'purpose':'Fresh closed-loop mission recording' if not args.preview_seconds else 'Camera framing preview only',
        'gazebo':subprocess.check_output(['gz','sim','--versions'],text=True).strip(),
        'ros_distro':os.environ.get('ROS_DISTRO'),'gz_partition':os.environ['GZ_PARTITION'],'ros_domain_id':'83',
        'upstream_commit':subprocess.check_output(['git','-C',str(ROOT/'vendor/bluerov2_gz'),'rev-parse','HEAD'],text=True).strip(),
        'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                        for folder in ['rov_survey','scripts','config'] for p in (ROOT/folder).rglob('*')
                        if p.is_file() and '__pycache__' not in str(p)},
        'world_sha256':hashlib.sha256((out/'world.sdf').read_bytes()).hexdigest(),
        'camera_note':'Only render sensors added; vehicle dynamics and controller unchanged. Chase sensor is attached to base_link; vehicle pose is never set.'}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    processes=[]; streams=[]; result=1
    def start(name,command):
        log=(out/(name+'.log')).open('w'); streams.append(log)
        proc=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc); return proc
    try:
        gazebo=start('gazebo',['gz','sim','-s','--headless-rendering','-v','3','--seed',str(scenario['seed']),str(out/'world.sdf')])
        bridge=start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p',f'config_file:={ROOT / "config/bridge.yaml"}'])
        controller=start('controller',[sys.executable,'-m','rov_survey.node','--config',str(out/'mission.json'),'--output',str(out)])
        ready=False; deadline=time.monotonic()+50
        while time.monotonic()<deadline:
            if gazebo.poll() is not None: raise RuntimeError('Gazebo startup failed; inspect gazebo.log')
            response=subprocess.run(['gz','service','-s','/world/survey/control','--reqtype','gz.msgs.WorldControl',
                '--reptype','gz.msgs.Boolean','--timeout','1000','--req','pause: false'],capture_output=True,text=True)
            if 'data: true' in response.stdout: ready=True; break
            time.sleep(.5)
        if not ready: raise RuntimeError('Could not unpause world')
        print(f'Recording Gazebo cameras: {out}',flush=True)
        deadline=time.monotonic()+1800; last_status_update=0
        while time.monotonic()<deadline:
            if gazebo.poll() is not None or bridge.poll() is not None: raise RuntimeError('Gazebo or bridge exited')
            if any(rec.error for rec in recordings): raise RuntimeError('Video encoder failed')
            try:
                status=json.loads((out/'status.json').read_text()) if (out/'status.json').exists() else {}
            except json.JSONDecodeError:
                time.sleep(.05); continue
            stamp=status.get('simulation_time',0)
            if time.monotonic()-last_status_update>20:
                print(f't={stamp:.1f}s; legs={len(status.get("completed_legs",[]))}/9; camera frames={[r.written for r in recordings]}',flush=True)
                last_status_update=time.monotonic()
            if args.preview_seconds and min(r.last_time or 0 for r in recordings)>=args.preview_seconds:
                result=0; break
            if controller.poll() is not None:
                if status.get('state')!='COMPLETED': raise RuntimeError('Mission did not complete; inspect controller.log')
                if all((rec.last_time or 0)>=stamp+.1 for rec in recordings): result=0; break
            time.sleep(.15)
        else: raise TimeoutError('Mission recording deadline exceeded')
    finally:
        for proc in reversed(processes):
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
        for proc in processes:
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL); proc.wait()
        for stream in streams: stream.close()
        captures={rec.name:rec.close() for rec in recordings}
        (out/'capture.json').write_text(json.dumps({'cameras':captures,'playback':'1 second of video corresponds to 1 second of simulation time','world':'world.sdf'},indent=2)+'\n')
    return result


if __name__=='__main__':
    raise SystemExit(main())
