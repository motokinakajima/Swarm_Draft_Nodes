#!/usr/bin/env python3
"""
Standalone sanity check: does nothing but fetch real satellite imagery for
the temperature field's actual lat/lon footprint and paint the field on top,
then save a single PNG. No bag, no robots, no local-frame conversions - just
"is this field where I think it is on a real map".

Usage:
  python3 render_field_map.py [path/to/field.mat] [-o out.png]
"""
import argparse
import io
import math
from pathlib import Path

import numpy as np
import scipy.io
import requests
from PIL import Image

TILE_SIZE = 256
TILE_URL = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'


def lonlat_to_px(lon, lat, zoom):
    scale = TILE_SIZE * 2 ** zoom
    x = (lon + 180.0) / 360.0 * scale
    sin_lat = math.sin(math.radians(lat))
    y = (0.5 - math.log((1 + sin_lat) / (1 - sin_lat)) / (4 * math.pi)) * scale
    return x, y


def pick_zoom(lat_span_m, lon_span_m, center_lat, max_tiles_across=6):
    for zoom in range(19, 0, -1):
        mpp = 156543.03392 * math.cos(math.radians(center_lat)) / (2 ** zoom)
        if max(lat_span_m, lon_span_m) / mpp <= max_tiles_across * TILE_SIZE:
            return zoom
    return 1


def fetch_tile(zoom, tx, ty, session):
    url = TILE_URL.format(z=zoom, y=ty, x=tx)
    resp = session.get(url, timeout=10)
    if resp.status_code != 200:
        print(f"  [tile z={zoom} x={tx} y={ty}] HTTP {resp.status_code}")
        return None
    return Image.open(io.BytesIO(resp.content)).convert('RGB')


def coolwarm(t):
    if t < 0.5:
        u = t / 0.5
        return (int(59 + (245 - 59) * u), int(76 + (245 - 76) * u), int(192 + (245 - 192) * u))
    u = (t - 0.5) / 0.5
    return (int(245 + (200 - 245) * u), int(245 + (60 - 245) * u), int(245 + (60 - 245) * u))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mat_path', nargs='?', default='../fake_tempratures/wilsons_landing_smaller_multimodal.mat')
    ap.add_argument('-o', '--out', default='field_on_satellite.png')
    ap.add_argument('--alpha', type=int, default=140, help='Heatmap opacity 0-255 (default 140)')
    args = ap.parse_args()

    data = scipy.io.loadmat(args.mat_path)
    lat_mesh = data['latMesh']
    lon_mesh = data['lonMesh']
    z_mean = data['zMean']
    rows, cols = lat_mesh.shape

    lat_min, lat_max = float(lat_mesh.min()), float(lat_mesh.max())
    lon_min, lon_max = float(lon_mesh.min()), float(lon_mesh.max())
    center_lat = (lat_min + lat_max) / 2
    print(f"Field covers lat [{lat_min:.6f}, {lat_max:.6f}], lon [{lon_min:.6f}, {lon_max:.6f}]")

    lat_span_m = (lat_max - lat_min) * 111_320
    lon_span_m = (lon_max - lon_min) * 111_320 * math.cos(math.radians(center_lat))
    zoom = pick_zoom(lat_span_m, lon_span_m, center_lat)
    print(f"Using zoom {zoom}")

    px0, py0 = lonlat_to_px(lon_min, lat_max, zoom)  # NW corner
    px1, py1 = lonlat_to_px(lon_max, lat_min, zoom)  # SE corner

    tx0, tx1 = int(px0 // TILE_SIZE), int(px1 // TILE_SIZE)
    ty0, ty1 = int(py0 // TILE_SIZE), int(py1 // TILE_SIZE)
    nx, ny = tx1 - tx0 + 1, ty1 - ty0 + 1
    print(f"Fetching {nx}x{ny} = {nx * ny} tiles...")

    mosaic = Image.new('RGB', (nx * TILE_SIZE, ny * TILE_SIZE), (30, 30, 30))
    session = requests.Session()
    ok = 0
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            tile = fetch_tile(zoom, tx, ty, session)
            if tile is not None:
                mosaic.paste(tile, ((tx - tx0) * TILE_SIZE, (ty - ty0) * TILE_SIZE))
                ok += 1
    print(f"Got {ok}/{nx * ny} tiles successfully.")
    if ok == 0:
        raise SystemExit(
            "No tiles loaded at all - this is a network/connectivity problem talking to "
            "server.arcgisonline.com from this machine, not a georeferencing bug. "
            "Try 'curl -I https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/1/0/0' "
            "to confirm."
        )

    # Crop the mosaic to exactly the field's bounding box (tiles cover a
    # slightly larger area than the field itself).
    crop_left = int(px0 - tx0 * TILE_SIZE)
    crop_top = int(py0 - ty0 * TILE_SIZE)
    crop_right = int(px1 - tx0 * TILE_SIZE)
    crop_bottom = int(py1 - ty0 * TILE_SIZE)
    base = mosaic.crop((crop_left, crop_top, crop_right, crop_bottom))
    out_w, out_h = base.size
    print(f"Cropped basemap to field bounds: {out_w}x{out_h}px")

    # Build the heatmap layer at the field's native resolution then upscale
    # to the basemap's pixel size with nearest-neighbor so each Gaussian
    # cell stays crisp (bilinear would smear it into the wrong shape).
    zmin, zmax = float(z_mean.min()), float(z_mean.max())
    heat = Image.new('RGBA', (cols, rows))
    pixels = heat.load()
    for r in range(rows):
        for c in range(cols):
            t = (z_mean[r, c] - zmin) / (zmax - zmin + 1e-9)
            rr, gg, bb = coolwarm(t)
            # row 0 of the .mat grid is the southernmost latitude, but image
            # row 0 is the top (north) of the picture - flip vertically.
            pixels[c, rows - 1 - r] = (rr, gg, bb, args.alpha)
    heat = heat.resize((out_w, out_h), Image.NEAREST)

    composed = base.convert('RGBA')
    composed.alpha_composite(heat)
    composed = composed.convert('RGB')
    composed.save(args.out)
    print(f"Saved {args.out} ({out_w}x{out_h})")


if __name__ == '__main__':
    main()
