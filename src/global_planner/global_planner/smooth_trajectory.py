#!/usr/bin/env python3
"""
Etapa 2: suaviza la ruta cruda con una B-Spline cubica y verifica viabilidad.

Salidas en waypoints/:
  dijkstra_waypoints_smooth.csv        (x, y, yaw, curvature)
  trayectoria_suavizada.png
  comparacion_crudo_vs_suavizado.png
  curvatura.png
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
    ap.add_argument('--out', default=pu.pkg_dir('waypoints'))
    ap.add_argument('--step', type=float, default=0.30,
                    help='separacion de los puntos de control [m]')
    args = ap.parse_args()

    cache_f = os.path.join(args.out, '_planner_cache.npz')
    if not os.path.exists(cache_f):
        raise SystemExit('[X] Falta %s. Corre primero generate_trajectory.' % cache_f)
    cache = np.load(cache_f, allow_pickle=True)

    meta, occupied, free, unknown = pu.load_map(str(cache['map_yaml']))
    H = occupied.shape[0]
    res = float(meta['resolution'])
    track, _, _ = pu.find_track(free, seed=tuple(int(v) for v in cache['seed']))
    dist, drivable, _ = pu.build_cost_field(
        track, res, float(cache['clearance']), float(cache['penalty']), float(cache['d_ref']))

    raw_px = cache['raw_px']
    ckpts = cache['ckpts']
    closed = bool(cache['closed'])
    raw_xy = pu.path_to_world(raw_px, meta, H)

    print('[1] Ruta cruda: %d nodos, %.2f m (%s)'
          % (len(raw_xy), pu.path_length(raw_xy), 'lazo cerrado' if closed else 'abierta'))
    print('[2] Escalera de suavizado B-Spline (se toma el primero ACEPTADO):')
    smooth_xy, yaw, curv, s_used, ncol, kmax = pu.smooth_path_safe(
        raw_xy, res, drivable, meta, H, closed, step_m=args.step)

    n_raw_col = pu.count_collisions(raw_xy, drivable, meta, H)
    L_raw, L_smo = pu.path_length(raw_xy), pu.path_length(smooth_xy)

    print('[3] Resultados:')
    print('    Factor s elegido      : %.2f' % s_used)
    print('    Longitud cruda        : %.2f m' % L_raw)
    print('    Longitud suavizada    : %.2f m' % L_smo)
    print('    Curvatura maxima      : %.3f 1/m  (limite F1TENTH %.3f 1/m)'
          % (kmax, pu.KAPPA_MAX))
    print('    Radio de giro minimo  : %.2f m' % (1.0 / max(kmax, 1e-9)))
    print('    Colisiones cruda      : %d' % n_raw_col)
    print('    Colisiones suavizada  : %d' % ncol)
    print('    Viabilidad cinematica : %s'
          % ('CUMPLE' if kmax <= pu.KAPPA_MAX else 'EXCEDE -> sube --step'))

    f_csv = os.path.join(args.out, 'dijkstra_waypoints_smooth.csv')
    np.savetxt(f_csv, np.column_stack([smooth_xy[:, 0], smooth_xy[:, 1], yaw, curv]),
               delimiter=',', header='x,y,yaw,curvature', comments='', fmt='%.6f')
    print('[4] CSV ->', f_csv)

    smooth_px = pu.path_to_grid(smooth_xy, meta, H)
    rgb = pu.canvas(occupied, drivable, track)
    cp = np.array(ckpts)

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(rgb)
    ax.plot(raw_px[:, 1], raw_px[:, 0], '--', color='0.55', lw=1.0, label='Ruta cruda')
    ax.plot(smooth_px[:, 1], smooth_px[:, 0], '-', color='seagreen', lw=2,
            label='Ruta suavizada (B-Spline)')
    ax.plot(cp[0, 1], cp[0, 0], 'o', color='limegreen', ms=11, label='Inicio')
    ax.set_title('Trayectoria suavizada: kappa_max %.3f 1/m, %.2f m' % (kmax, L_smo))
    ax.set_axis_off()
    ax.legend(loc='lower right', fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'trayectoria_suavizada.png'), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(14, 7))
    for a in ax:
        a.imshow(rgb)
        a.set_axis_off()
    ax[0].plot(raw_px[:, 1], raw_px[:, 0], '-', color='orangered', lw=2)
    ax[0].set_title('Cruda (Dijkstra): %d nodos, %.2f m' % (len(raw_px), L_raw))
    ax[1].plot(raw_px[:, 1], raw_px[:, 0], '--', color='0.6', lw=1)
    ax[1].plot(smooth_px[:, 1], smooth_px[:, 0], '-', color='seagreen', lw=2)
    ax[1].set_title('Suavizada (B-Spline): %.2f m, %d colisiones' % (L_smo, ncol))
    fig.suptitle('Comparacion: trayectoria cruda vs. suavizada', fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'comparacion_crudo_vs_suavizado.png'), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 3.4))
    ax.plot(np.linspace(0, L_smo, len(curv)), curv, color='navy', lw=1.4,
            label='Curvatura de la B-Spline')
    ax.axhline(pu.KAPPA_MAX, color='red', ls='--',
               label='Limite F1TENTH: tan(30 deg)/L = %.2f 1/m' % pu.KAPPA_MAX)
    ax.set_xlabel('Distancia recorrida [m]')
    ax.set_ylabel('Curvatura [1/m]')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'curvatura.png'), dpi=150)
    plt.close(fig)

    print('    PNG -> trayectoria_suavizada.png')
    print('    PNG -> comparacion_crudo_vs_suavizado.png')
    print('    PNG -> curvatura.png')
    print('\n[OK] Etapa 2 completa. Sigue: ros2 run global_planner generate_gif')


if __name__ == '__main__':
    main()
