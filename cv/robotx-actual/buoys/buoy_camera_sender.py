#!/home/optiprime-agxorin/actual_buoy/.venv/bin/python
"""Jetson 1: detect the nearest buoy, measure ZED depth, and send signed UDP observations."""

import argparse, hashlib, hmac, json, socket, time, uuid
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO

from actual_buoy_observer import measure_mask, open_zed


def args_parse():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights',default='/home/optiprime-agxorin/actual_buoy/best.pt')
    p.add_argument('--key-file',default='/home/optiprime-agxorin/actual_buoy/buoy_link.key')
    p.add_argument('--destination',default='192.168.1.43',help='Orin 2 receiver IP')
    p.add_argument('--port',type=int,default=15150)
    p.add_argument('--resolution',choices=['hd2k','hd1080','hd720','vga'],default='hd720')
    p.add_argument('--fps',type=int,default=15)
    p.add_argument('--depth-mode',choices=['neural','neural_light','ultra'],default='neural')
    p.add_argument('--depth-max-m',type=float,default=20.0,
                   help='Maximum ZED depth search distance in metres (1..40; values above 20 may be unreliable)')
    p.add_argument('--conf',type=float,default=0.65)
    p.add_argument('--imgsz',type=int,default=768)
    p.add_argument('--frames',type=int,default=0)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--self-test',action='store_true')
    return p.parse_args()


def signed_packet(body,key):
    raw=json.dumps(body,separators=(',',':'),sort_keys=True).encode()
    return json.dumps({'body':body,'hmac_sha256':hmac.new(key,raw,hashlib.sha256).hexdigest()},separators=(',',':')).encode()


def verify_packet(packet,key):
    envelope=json.loads(packet)
    raw=json.dumps(envelope['body'],separators=(',',':'),sort_keys=True).encode()
    expected=hmac.new(key,raw,hashlib.sha256).hexdigest()
    return hmac.compare_digest(expected,envelope['hmac_sha256'])


def main():
    a=args_parse(); key=Path(a.key_file).read_text().strip().encode()
    if a.self_test:
        test=signed_packet({'protocol':'actual_buoy.v1','sequence':1},key)
        assert verify_packet(test,key); print('{"self_test":"PASS"}'); return
    (Path(__file__).resolve().parent/'runs').mkdir(exist_ok=True)
    session=str(uuid.uuid4()); sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    if not 1.0 <= a.depth_max_m <= 40.0:
        raise ValueError('--depth-max-m must be between 1 and 40 metres')
    model=YOLO(a.weights,task='segment'); camera=open_zed(a.resolution,a.fps,a.depth_mode,a.depth_max_m)
    runtime=sl.RuntimeParameters(); runtime.confidence_threshold=50; runtime.texture_confidence_threshold=100
    image,cloud=sl.Mat(),sl.Mat(); seq=0
    try:
        while not a.frames or seq<a.frames:
            if camera.grab(runtime)!=sl.ERROR_CODE.SUCCESS: time.sleep(.02); continue
            camera.retrieve_image(image,sl.VIEW.LEFT); camera.retrieve_measure(cloud,sl.MEASURE.XYZRGBA)
            frame=image.get_data()[:,:,:3].copy(); xyz=cloud.get_data()
            result=model.predict(frame,conf=a.conf,imgsz=a.imgsz,device=0,verbose=False)[0]
            candidates=[]
            if result.boxes is not None and result.masks is not None:
                masks=result.masks.data.cpu().numpy()
                for i,box in enumerate(result.boxes):
                    mask=cv2.resize(masks[i],(frame.shape[1],frame.shape[0]),interpolation=cv2.INTER_LINEAR)
                    measurement=measure_mask(xyz,mask,min_valid_fraction=.35,max_depth_m=a.depth_max_m)
                    if measurement: candidates.append((measurement.range_m,float(box.conf.item()),measurement,box))
            candidates.sort(key=lambda x:x[0])
            target=candidates[0] if candidates else None
            body={'protocol':'actual_buoy.v1','session':session,'sequence':seq,
                  'sent_unix_ns':time.time_ns(),'sent_utc':datetime.now(timezone.utc).isoformat(),
                  'camera_host':socket.gethostname(),'status':'OK' if target else 'NO_RELIABLE_BUOY',
                  'target':None,'targets':[],'primary_target_index':None}
            for buoy_id,(_,conf,m,box) in enumerate(candidates):
                item={'buoy_id':buoy_id,'confidence':conf,'range_m':m.range_m,
                      'forward_m':m.forward_m,'right_m':m.right_m,'down_m':m.down_m,
                      'bearing_deg':m.bearing_deg,'valid_pixels':m.valid_pixels,
                      'valid_fraction':m.valid_fraction,'depth_spread_m':m.depth_spread_m}
                body['targets'].append(item)
                x1,y1,x2,y2=map(int,box.xyxy[0].cpu().numpy()); cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(frame,f"buoy {buoy_id} | {m.range_m:.2f}m | {m.bearing_deg:+.1f}deg",(max(0,x1),max(35,y1-10)),0,.65,(0,255,0),2)
            if target:
                body['target']=body['targets'][0]
                body['primary_target_index']=0
            sock.sendto(signed_packet(body,key),(a.destination,a.port))
            if seq==0 or seq%15==0:
                print(f"seq={seq} status={body['status']} buoys={len(body['targets'])} primary={body['target']} dest={a.destination}:{a.port}",flush=True)
                cv2.imwrite(str(Path(__file__).resolve().parent/'runs'/'latest_sender_preview.jpg'),frame)   # snapshot: our runs/, not the students' folder
            if not a.headless:
                cv2.imshow('Buoy camera sender - Q/Esc',frame)
                if cv2.waitKey(1)&255 in (27,ord('q')): break
            seq+=1
    finally:
        camera.close(); sock.close()
        if not a.headless: cv2.destroyAllWindows()

if __name__=='__main__': main()
