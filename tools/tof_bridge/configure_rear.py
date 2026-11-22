#!/usr/bin/env python3
"""Save/read back the requested rear MAVLink rangefinder setup, optionally reboot.

Uses unused rangefinder slot 2 and disables the earlier proximity backend.
Rangefinder slot 1 is preserved. Never arms or
changes flight modes. A reboot is refused unless a fresh disarmed heartbeat
was received. Run with the viewer stopped so UART ownership is exclusive.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import serial
from pymavlink.dialects.v20 import ardupilotmega as mav
from live_viewer import detect_baud

DESIRED = {'PRX1_TYPE': 0, 'RNGFND2_TYPE': 10, 'RNGFND2_ORIENT': 4,
           'RNGFND2_MIN': 0.01, 'RNGFND2_MAX': 5.0}


def same(a,b):
    return math.isclose(a,b,rel_tol=1e-6,abs_tol=1e-6)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--reboot', action='store_true')
    parser.add_argument('--uart', default='/dev/ttyTHS1')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.reboot and not args.apply:
        parser.error('--reboot requires --apply')
    args.output.mkdir(parents=True, exist_ok=False)
    result = {'utc':datetime.now(timezone.utc).isoformat(), 'before':{}, 'after':{},
              'writes':[], 'status_text':[], 'reboot_sent':False, 'error':None}

    def save():
        (args.output/'configuration.json').write_text(json.dumps(result,indent=2)+'\n')

    def receive(link, port):
        messages = link.parse_buffer(port.read(4096)) or []
        for msg in messages:
            if msg.get_srcSystem()==1 and msg.get_srcComponent()==1 and msg.get_type()=='STATUSTEXT':
                row = {'monotonic_s':time.monotonic(), 'severity':msg.severity, 'text':msg.text}
                result['status_text'].append(row)
                print('STATUSTEXT',json.dumps(row),flush=True)
        return messages

    def read_param(link, port, name, expected=None):
        deadline, next_request = time.monotonic()+4, 0
        while time.monotonic()<deadline:
            if time.monotonic()>=next_request:
                link.param_request_read_send(1,1,name.encode(),-1)
                next_request=time.monotonic()+.5
            for msg in receive(link,port):
                if msg.get_type()=='PARAM_VALUE' and msg.param_id==name and msg.get_srcSystem()==1 and msg.get_srcComponent()==1:
                    if expected is not None and not same(msg.param_value,expected):
                        continue  # Ignore queued replies from before PARAM_SET.
                    return {'value':msg.param_value,'type':msg.param_type}
        raise TimeoutError(f'No readback for {name}')

    def disarmed(link, port):
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            for msg in receive(link,port):
                if msg.get_type()=='HEARTBEAT' and msg.get_srcSystem()==1 and msg.get_srcComponent()==1:
                    if msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
                        raise RuntimeError('Cube is armed; refusing configuration/reboot')
                    return msg.to_dict()
        raise TimeoutError('No fresh autopilot heartbeat; refusing configuration/reboot')

    try:
        baud=detect_baud(SimpleNamespace(baud=None,uart=args.uart,system_id=1))
        result['initial_baud']=baud
        with serial.Serial(args.uart,baud,timeout=.05,write_timeout=1,exclusive=True) as port:
            link=mav.MAVLink(port,srcSystem=254,srcComponent=190);link.robust_parsing=True
            result['heartbeat_before']=disarmed(link,port)
            for name in (*DESIRED,'SERIAL1_PROTOCOL','SERIAL1_BAUD','BRD_SER1_RTSCTS',
                         'RNGFND1_TYPE','RNGFND1_ORIENT','RNGFND1_MIN','RNGFND1_MAX'):
                result['before'][name]=read_param(link,port,name)
            save()
            print('Original parameters saved:',args.output/'configuration.json',flush=True)
            if args.apply:
                if result['before']['RNGFND2_TYPE']['value'] not in (0,10):
                    raise RuntimeError('Rangefinder slot 2 is occupied; refusing to overwrite it')
                if result['before']['RNGFND2_TYPE']['value']==10 and result['before']['RNGFND2_ORIENT']['value']!=4:
                    raise RuntimeError('Slot 2 is configured for another direction; refusing to overwrite it')
                for name,value in DESIRED.items():
                    if not same(result['before'][name]['value'],value):
                        link.param_set_send(1,1,name.encode(),value,result['before'][name]['type'])
                        readback=read_param(link,port,name,expected=value)
                        if not same(readback['value'],value):
                            raise RuntimeError(f'{name} readback mismatch: {readback}')
                        result['writes'].append({'name':name,'old':result['before'][name]['value'],'new':value})
                        save()
                        print('Verified',name,'=',value,flush=True)
                if args.reboot:
                    result['heartbeat_before_reboot']=disarmed(link,port)
                    result['reboot_sent']=True
                    save()
                    link.command_long_send(1,1,mav.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,0,1,0,0,0,0,0,0)
                    port.flush()
                    print('Reboot requested from disarmed Cube',flush=True)
        if args.reboot:
            time.sleep(4)
            deadline=time.monotonic()+45
            while True:
                try:
                    baud=detect_baud(SimpleNamespace(baud=None,uart=args.uart,system_id=1))
                    break
                except Exception:
                    if time.monotonic()>deadline:raise
                    time.sleep(1)
        result['final_baud']=baud
        with serial.Serial(args.uart,baud,timeout=.05,write_timeout=1,exclusive=True) as port:
            link=mav.MAVLink(port,srcSystem=254,srcComponent=190);link.robust_parsing=True
            result['heartbeat_after']=disarmed(link,port)
            for name in (*DESIRED,'SERIAL1_PROTOCOL','SERIAL1_BAUD','RNGFND1_TYPE',
                         'RNGFND1_ORIENT','RNGFND1_MIN','RNGFND1_MAX'):
                result['after'][name]=read_param(link,port,name)
            if args.apply and any(not same(result['after'][n]['value'],v) for n,v in DESIRED.items()):
                raise RuntimeError('Final parameter verification failed')
            if any(not same(result['after'][n]['value'],result['before'][n]['value'])
                   for n in ('RNGFND1_TYPE','RNGFND1_ORIENT','RNGFND1_MIN','RNGFND1_MAX')):
                raise RuntimeError('Existing downward rangefinder settings changed during configuration')
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:receive(link,port)
        print('Final readback:',json.dumps(result['after']), 'baud:',baud,flush=True)
    except Exception as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
        print(result['error'],flush=True)
    finally:
        save()
    return 1 if result['error'] else 0


if __name__=='__main__':
    raise SystemExit(main())
