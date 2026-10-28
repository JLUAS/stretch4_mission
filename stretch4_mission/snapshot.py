#!/usr/bin/env python3
"""The picture after a policy: one frame per camera from the running stretch4_policy_stream, saved upright as JPEG.

    <recorder .venv>/bin/python snapshot.py --out DIR --cameras head_right gripper
        -> {"files": [...]} on the last line of stdout; errors: {"error": "..."} and exit code 1

Run by policy_bridge.py when a recorder policy stops, after stretch4_policy_exec and before stretch4_policy_stream: the
robot is still and the stream still sends. It only listens to the stream (port 5580), so it never opens a camera. It
needs the recorder's .venv (its stream decoding and upright rotation), so it runs as its own process.
"""

import argparse
import json
import sys
from pathlib import Path


def grab(out_dir, cameras, port, width):
    import cv2
    import numpy as np
    import zmq
    from stretch4_policy_recorder.common.image import preprocess
    from stretch4_policy_recorder.policy import remote

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b'')
    sock.setsockopt(zmq.RCVTIMEO, 5000)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f'tcp://127.0.0.1:{port}')
    try:
        header, heads, grip = remote.unpack_obs(sock.recv_multipart())
    except zmq.Again:
        raise RuntimeError(f'no images from stretch4_policy_stream on port {port} in 5 s: is it running?') from None
    finally:
        sock.close()
        ctx.term()
    jpegs = {**heads, 'gripper': grip}
    rots = {'gripper': header.get('grip_rot', 0), **{c: header.get(f'{c}_rot', 0) for c in heads}}
    missing = [c for c in cameras if c not in jpegs]
    if missing:
        raise RuntimeError(f"the stream has no {', '.join(missing)} camera (it sends: {', '.join(jpegs)})")
    out_dir.mkdir(parents=True, exist_ok=True)
    files = []
    for cam in cameras:
        bgr = cv2.imdecode(np.frombuffer(jpegs[cam], np.uint8), cv2.IMREAD_COLOR)
        rgb = preprocess(bgr, rots[cam], None, width)   # the recorder's own upright rotation
        path = out_dir / f'{cam}.jpg'
        cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
        files.append(str(path))
    return {'files': files}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--cameras', nargs='+', default=['head_right', 'gripper'])
    ap.add_argument('--port', type=int, default=5580)
    ap.add_argument('--width', type=int, default=640)
    a = ap.parse_args()
    try:
        out = grab(a.out, a.cameras, a.port, a.width)
    except (RuntimeError, ImportError, OSError, ValueError) as exc:
        print(json.dumps({'error': str(exc)}))
        return 1
    print(json.dumps(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
