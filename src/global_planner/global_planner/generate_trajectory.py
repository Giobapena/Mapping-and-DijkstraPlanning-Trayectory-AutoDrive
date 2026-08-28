#!/usr/bin/env python3
"""
Etapa 1: genera la trayectoria global CRUDA con Dijkstra.

Salidas en waypoints/:
  dijkstra_waypoints.csv     ruta cruda (x, y)
  trayectoria_cruda.png
  _planner_cache.npz         datos intermedios para las etapas 2 y 3
"""
import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from . import planning_utils as pu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.join(pu.pkg_dir('maps'), 'F1tenth_Map.yaml'))
    ap.add_argument('--out', default=pu.pkg_dir('waypoints'))
    ap.add_argument('--mode', choices=['lap', 'point'], default='lap',
                    help='lap = vuelta completa; point = inicio -> meta')
    ap.add_argument('--start', nargs=2, type=float, default=[0.744, 3.158],
                    metavar=('X', 'Y'), help='spawn del vehiculo (de /ips)')
    ap.add_argument('--goal', nargs=2, type=float, default=None, metavar=('X', 'Y'))
    ap.add_argument('--checkpoints', type=int, default=24,
                    help='con menos, la costura del lazo cerrado junto al '
                         'spawn no pasa la escalera de suavizado (ver README)')
    ap.add_argument('--clearance', type=float, default=0.22,
                    help='holgura minima a pared [m]')
    ap.add_argument('--penalty', type=float, default=3.0,
                    help='peso de la penalizacion por cercania a pared')
    ap.add_argument('--d-ref', type=float, default=0.60,
                    help='distancia [m] a partir de la cual ya no se penaliza')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # ---------------------------------------------------------------- mapa
    meta, occupied, free, unknown = pu.load_map(args.map)
    H, W = occupied.shape
    res = float(meta['resolution'])

    # La pista se identifica como la componente libre que CONTIENE al vehiculo,
    # no como la mas grande: el exterior del circuito tambien es espacio libre
    # y suele ser mayor que la pista misma.
    seed = pu.world_to_grid(args.start[0], args.start[1], meta, H)
    track, ncomp, sizes = pu.find_track(free, seed=seed)

    print('[1] Mapa %d x %d px | resolucion %.3f m/px' % (W, H, res))
    print('    Componentes libres detectadas : %d' % ncomp)
    print('    Tamano de las 3 mayores       : %s' % np.sort(sizes)[::-1][:3].astype(int))
    print('    Semilla (spawn) en celda      : %s' % (seed,))
    print('    Pista seleccionada            : %d celdas (%.1f m2)'
          % (track.sum(), track.sum() * res * res))

    dist, drivable, mult = pu.build_cost_field(
        track, res, args.clearance, args.penalty, args.d_ref)
    print('    Transitable (holgura >= %.2f m): %d celdas' % (args.clearance, drivable.sum()))
    if drivable.sum() < 200:
        raise SystemExit('[X] Casi no queda zona transitable. Baja --clearance.')

    snap = pu.make_snapper(drivable)

    # ------------------------------------------------------- inicio y meta
    start_cell = snap(pu.world_to_grid(args.start[0], args.start[1], meta, H))
    sx, sy = pu.grid_to_world(*start_cell, meta=meta, H=H)
    print('[2] Inicio: mundo (%.3f, %.3f) -> celda %s | holgura %.2f m'
          % (sx, sy, start_cell, dist[start_cell]))

    # -------------------------------------------------------- planificacion
    closed = (args.mode == 'lap')
    if closed:
        ckpts, center = pu.lap_checkpoints(drivable, dist, start_cell, args.checkpoints)
        print('[3] Vuelta completa: %d checkpoints (centroide fila %.0f col %.0f)'
              % (len(ckpts), center[0], center[1]))
        raw_px, order, cost = pu.plan_lap(drivable, mult, ckpts, record=True)
    else:
        if args.goal is None:
            raise SystemExit('[X] El modo point requiere --goal X Y')
        goal_cell = snap(pu.world_to_grid(args.goal[0], args.goal[1], meta, H))
        gx, gy = pu.grid_to_world(*goal_cell, meta=meta, H=H)
        print('[3] Meta: mundo (%.3f, %.3f) -> celda %s' % (gx, gy, goal_cell))
        ckpts = [start_cell, goal_cell]
        raw_px, order, cost = pu.dijkstra(drivable, mult, start_cell, goal_cell, record=True)

    raw_xy = pu.path_to_world(raw_px, meta, H)
    ncol = pu.count_collisions(raw_xy, drivable, meta, H)

    print('    Nodos expandidos por Dijkstra : %d' % len(order))
    print('    Nodos de la ruta resultante   : %d' % len(raw_px))
    print('    Costo total (celdas ponderadas): %.1f' % cost)
    print('    Longitud de la ruta cruda     : %.2f m' % pu.path_length(raw_xy))
    print('    Colisiones                    : %d' % ncol)

    # -------------------------------------------------------------- salidas
    f_csv = os.path.join(args.out, 'dijkstra_waypoints.csv')
    np.savetxt(f_csv, raw_xy, delimiter=',', header='x,y', comments='', fmt='%.6f')
    print('[4] CSV ->', f_csv)

    np.savez_compressed(
        os.path.join(args.out, '_planner_cache.npz'),
        raw_px=raw_px, order=np.array(order, dtype=np.int32),
        ckpts=np.array(ckpts, dtype=np.int32), closed=closed,
        seed=np.array(seed, dtype=np.int32),
        map_yaml=os.path.abspath(args.map), clearance=args.clearance,
        penalty=args.penalty, d_ref=args.d_ref)

    rgb = pu.canvas(occupied, drivable, track)
    cp = np.array(ckpts)
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(rgb)
    ax.plot(raw_px[:, 1], raw_px[:, 0], '-', color='orangered', lw=2,
            label='Ruta cruda (Dijkstra)')
    ax.plot(cp[1:, 1], cp[1:, 0], 'o', color='royalblue', ms=5, label='Checkpoints')
    ax.plot(cp[0, 1], cp[0, 0], 'o', color='limegreen', ms=11, label='Inicio (spawn)')
    ax.set_title('Trayectoria cruda: %d nodos, %.2f m' % (len(raw_px), pu.path_length(raw_xy)))
    ax.set_axis_off()
    ax.legend(loc='lower right', fontsize=9)
    fig.tight_layout()
    f_png = os.path.join(args.out, 'trayectoria_cruda.png')
    fig.savefig(f_png, dpi=150)
    plt.close(fig)
    print('    PNG ->', f_png)
    print('\n[OK] Etapa 1 completa. Sigue: ros2 run global_planner smooth_trajectory')


if __name__ == '__main__':
    main()
