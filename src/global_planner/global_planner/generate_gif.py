#!/usr/bin/env python3
"""Etapa 3: anima la expansion de nodos de Dijkstra -> waypoints/dijkstra_search.gif"""
import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from . import planning_utils as pu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=pu.pkg_dir('waypoints'))
    ap.add_argument('--frames', type=int, default=90)
    ap.add_argument('--fps', type=int, default=12)
    args = ap.parse_args()

    cache_f = os.path.join(args.out, '_planner_cache.npz')
    if not os.path.exists(cache_f):
        raise SystemExit('[X] Falta %s. Corre primero generate_trajectory.' % cache_f)
    cache = np.load(cache_f, allow_pickle=True)

    meta, occupied, free, unknown = pu.load_map(str(cache['map_yaml']))
    track, _, _ = pu.find_track(free, seed=tuple(int(v) for v in cache['seed']))
    _, drivable, _ = pu.build_cost_field(
        track, float(meta['resolution']), float(cache['clearance']),
        float(cache['penalty']), float(cache['d_ref']))

    raw_px, order, ckpts = cache['raw_px'], cache['order'], cache['ckpts']
    print('[1] %d nodos expandidos -> %d fotogramas' % (len(order), args.frames))

    frame = pu.canvas(occupied, drivable, track)
    step = max(1, len(order) // args.frames)
    chunks = [order[i:i + step] for i in range(0, len(order), step)]

    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    im = ax.imshow(frame)
    ln, = ax.plot([], [], '-', color='orangered', lw=2)
    ax.plot(ckpts[:, 1], ckpts[:, 0], 'o', color='royalblue', ms=4)
    ax.plot(ckpts[0, 1], ckpts[0, 0], 'o', color='limegreen', ms=10)
    ax.set_title('Dijkstra: expansion de nodos y ruta resultante')
    ax.set_axis_off()

    def update(k):
        if k < len(chunks):
            for (r, c) in chunks[k]:
                frame[r, c] = [0.20, 0.52, 0.92]
            im.set_data(frame)
        else:
            ln.set_data(raw_px[:, 1], raw_px[:, 0])
        return im, ln

    ani = FuncAnimation(fig, update, frames=len(chunks) + 15, blit=False)
    f_gif = os.path.join(args.out, 'dijkstra_search.gif')
    ani.save(f_gif, writer=PillowWriter(fps=args.fps))
    plt.close(fig)
    print('[2] GIF ->', f_gif)
    print('\n[OK] Etapa 3 completa.')


if __name__ == '__main__':
    main()
