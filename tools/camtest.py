"""Exercise the panel's camera entity the way Home Assistant does, over the native API.

    .venv/bin/python tools/camtest.py [stills] [stream_seconds] [outdir] [gap_seconds]

Requests `stills` single images (spaced `gap_seconds` apart - 8 or more makes
the panel close and reopen the pipeline between them), writes each to `outdir`
as still_N.jpg, then holds a stream open for `stream_seconds` the way Home
Assistant does and reports the frame rate. Reads the API key from secrets.yaml,
so run it from the project root. Needs aioesphomeapi, which esphome installs.
"""
import asyncio, re, sys, time
from pathlib import Path
from aioesphomeapi import APIClient, CameraState

key = re.search(r'api_encryption_key:\s*"?([^"\n]+)"?', open('secrets.yaml').read()).group(1).strip()
STILLS = int(sys.argv[1]) if len(sys.argv) > 1 else 5
STREAM_S = float(sys.argv[2]) if len(sys.argv) > 2 else 20
OUT = Path(sys.argv[3] if len(sys.argv) > 3 else '.')
GAP = float(sys.argv[4]) if len(sys.argv) > 4 else 4


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


async def main():
    cli = APIClient("espdash.local", 6053, None, noise_psk=key)
    await cli.connect(login=True)
    info = await cli.device_info()
    log(f"connected to {info.name} ({info.esphome_version})")

    frames = []
    got = asyncio.Event()

    def on_state(state):
        if isinstance(state, CameraState):
            frames.append(state.data)
            got.set()

    cli.subscribe_states(on_state)

    ok = 0
    for i in range(STILLS):
        got.clear()
        t0 = time.monotonic()
        cli.request_single_image()
        try:
            await asyncio.wait_for(got.wait(), 15)
        except asyncio.TimeoutError:
            log(f"still {i + 1}: no image within 15 s")
            continue
        data = frames[-1]
        dt = time.monotonic() - t0
        good = data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9"
        ok += good
        path = OUT / f"still_{i + 1}.jpg"
        path.write_bytes(data)
        log(f"still {i + 1}: {len(data)} bytes in {dt:.2f} s, valid JPEG {good} -> {path}")
        await asyncio.sleep(GAP)
    log(f"stills: {ok}/{STILLS} valid")

    if STREAM_S > 0:
        n0 = len(frames)
        t0 = time.monotonic()
        last_req = 0
        while time.monotonic() - t0 < STREAM_S:
            # Home Assistant re-requests the stream every few seconds; the API
            # stops it 5 s after the last request.
            if time.monotonic() - last_req > 3:
                cli.request_image_stream()
                last_req = time.monotonic()
            await asyncio.sleep(0.2)
        n = len(frames) - n0
        total = sum(len(f) for f in frames[n0:])
        log(f"stream: {n} frames in {STREAM_S:.0f} s ({n / STREAM_S:.2f} fps), {total / 1024:.0f} KB total")
        if n:
            (OUT / "stream_last.jpg").write_bytes(frames[-1])
        await asyncio.sleep(8)  # let the device close the session

    await cli.disconnect()


asyncio.run(main())
