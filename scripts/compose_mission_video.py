#!/usr/bin/env python3

import argparse
import csv
import json
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
NAVY='#102637'; TEAL='#1fc3c6'; GOLD='#ffc35b'; RED='#ee795f'
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONTS={n:ImageFont.truetype(FONT,n) for n in [13,15,17,19,23,28,38]}


def decoder(path):
    return subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-i',str(path),
                             '-f','rawvideo','-pix_fmt','rgb24','pipe:1'],stdout=subprocess.PIPE)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,default=ROOT/'results/video_run_20261021')
    parser.add_argument('--output',type=Path,default=ROOT/'videos/ROV_Survey_Full_Run.mp4')
    args=parser.parse_args(); run=args.run
    args.output.parent.mkdir(parents=True,exist_ok=True)
    capture=json.loads((run/'capture.json').read_text())['cameras']
    status=json.loads((run/'status.json').read_text()); metrics=json.loads((run/'metrics.json').read_text())
    assert status['state']=='COMPLETED' and len(status['completed_legs'])==9
    assert min(c['last_sim_time_s'] for c in capture.values())>=status['simulation_time']
    rows=list(csv.DictReader((run/'telemetry.csv').open()))
    t=np.array([float(r['t']) for r in rows]); xy=np.array([[float(r['x']),float(r['y'])] for r in rows])
    plans=[json.loads(line) for line in (run/'plans.jsonl').read_text().splitlines()]
    fps=capture['overview']['fps']; count=min(c['frames'] for c in capture.values())
    start=capture['overview']['first_sim_time_s']
    times=[float(r['source_simulation_time_s']) for r in csv.DictReader((run/'overview_frames.csv').open())]
    readers=[decoder(run/(name+'_raw.mp4')) for name in ['overview','chase']]
    log=(run/'composite_ffmpeg.log').open('w')
    writer=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','rawvideo',
        '-pix_fmt','rgb24','-s','1920x1080','-framerate',str(fps),'-i','pipe:0','-an',
        '-c:v','libx264','-threads','4','-preset','medium','-crf','18','-pix_fmt','yuv420p',
        '-movflags','+faststart',str(args.output)],stdin=subprocess.PIPE,stdout=log,stderr=log)
    def point(p): return (1355+(p[0]+.6)*51,985-(p[1]+.6)*51)
    def text(draw,p,value,size=17,fill='white'): draw.text(p,value,font=FONTS[size],fill=fill)
    last=None
    try:
        for k in range(count):
            now=times[k]
            first=readers[0].stdout.read(1280*1080*3); second=readers[1].stdout.read(640*480*3)
            if len(first)!=1280*1080*3 or len(second)!=640*480*3: raise RuntimeError('Camera stream truncated')
            frame=Image.new('RGB',(1920,1080),NAVY)
            frame.paste(Image.frombytes('RGB',(1280,1080),first),(0,0))
            frame.paste(Image.frombytes('RGB',(640,480),second),(1280,0))
            d=ImageDraw.Draw(frame)
            d.rectangle((0,0,1280,72),fill=NAVY)
            text(d,(26,9),'BLUEROV2  |  SONAR-GUIDED SURVEY',28)
            text(d,(28,45),'Gazebo overhead view  ·  21 Oct 2026  ·  current (0.12, 0.08) m/s',17)
            d.rectangle((1280,0,1920,34),fill=NAVY)
            text(d,(1298,7),'GAZEBO / VEHICLE VIEW',17)
            text(d,(1300,488),'ACTIVE PLAN AND EXECUTION',19)
            text(d,(1300,517),'Log overlay: sonar map  /  plan  /  executed path',13,fill='#bfced7')
            for y in range(0,9,2):
                d.line([point((0,y)),point((8,y))],fill='#455968',width=1)
                text(d,(1300,point((0,y))[1]-8),f'L{y//2+1}',13,fill='#bfced7')
            d.rectangle((*point((0,8)),*point((8,0))),outline='#70828e',width=1)
            for x in range(0,9,2): text(d,(point((x,0))[0]-6,994),str(x),13,fill='#bfced7')
            valid=[p for p in plans if p['t']<=now]
            if valid:
                p=valid[-1]
                for center in p['centers']:
                    cx,cy=point(center); r=.25*51
                    d.ellipse((cx-r,cy-r,cx+r,cy+r),fill=RED)
                if len(p['path'])>1: d.line([point(v) for v in p['path']],fill=GOLD,width=3)
            i=min(max(int(np.searchsorted(t,now,side='right')-1),0),len(rows)-1)
            if i>0: d.line([point(v) for v in xy[:i+1:3]],fill=TEAL,width=3)
            cx,cy=point(xy[i]); d.ellipse((cx-6,cy-6,cx+6,cy+6),fill='white')
            legs=[v for v in status['completed_legs'] if v['time']<=now]
            lines=sum(v['name'].startswith('line_') for v in legs)
            leg=int(float(rows[i]['leg']))
            label='COMPLETED' if len(legs)==9 else (f'Survey line {leg//2+1}' if leg%2==0 else f'Connector {leg//2+1}')
            d.rectangle((0,1020,1920,1080),fill=NAVY)
            text(d,(26,1030),f't = {now:6.2f} s   |   {label}   |   Legs {len(legs)}/9   |   Line endpoints {lines}/5   |   Depth {-float(rows[i]["z"]):.2f} m',23)
            text(d,(26,1061),'Actual closed-loop simulation · six physical thrusters · playback: 1 s video = 1 s simulation',13,fill='#bfced7')
            writer.stdin.write(frame.tobytes()); last=frame
            if k%450==0: print(f'Composed {k}/{count} frames',flush=True)
        final=last.copy(); d=ImageDraw.Draw(final)
        d.rounded_rectangle((260,390,1050,650),radius=20,fill=NAVY,outline=TEAL,width=3)
        text(d,(300,423),'MISSION COMPLETED',38)
        text(d,(300,486),'9/9 ordered legs · all five line endpoints',23)
        text(d,(300,526),f'Simulation duration: {metrics["duration_sim_s"]:.2f} s   |   Travel: {metrics["distance_travelled_3d_m"]:.2f} m',19)
        text(d,(300,564),'Obstacle bypasses included; completion is not 100% coverage.',17)
        text(d,(300,602),'Final frame held for summary',15,fill='#bfced7')
        final.save(args.output.with_suffix('.png'))
        for _ in range(3*fps): writer.stdin.write(final.tobytes())
    finally:
        writer.stdin.close(); writer.wait(timeout=120); log.close()
        for reader in readers:
            reader.stdout.close(); reader.wait(timeout=20)
    if writer.returncode: raise RuntimeError('Composite encoding failed')
    meta={'run':str(run.relative_to(ROOT)),'output':str(args.output.relative_to(ROOT)),
          'dimensions':[1920,1080],'fps':fps,'mission_camera_frames':count,'summary_hold_s':3,
          'simulation_start_s':start,'simulation_final_camera_s':times[-1],
          'mission_completion_s':status['simulation_time'],'completed_legs':9,
          'camera_frame_repeats':{k:v['repeated_frames'] for k,v in capture.items()},
          'overlay_source':'This recording run: telemetry.csv, plans.jsonl and status.json'}
    args.output.with_suffix('.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(args.output)


if __name__=='__main__': main()
