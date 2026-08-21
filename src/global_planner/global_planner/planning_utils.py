#!/usr/bin/env python3
"""
Nucleo de planificacion global sobre el mapa de ocupacion de SLAM Toolbox.

Contenido:
  - Carga del mapa trinario (.pgm + .yaml) y conversion pixel <-> mundo.
  - Extraccion de la pista real por componente conexa mas grande.
  - Campo de costo por cercania a pared (transformada de distancia).
  - Dijkstra 8-conectado con costo ponderado.
  - Reparto de checkpoints para cubrir la vuelta completa.
  - B-Spline cubica con busqueda de suavizado seguro.
  - Verificacion cinematica por curvatura para el F1TENTH.
"""
import heapq
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splprep, splev

SQ2 = math.sqrt(2.0)

# Parametros fisicos del F1TENTH en AutoDRIVE
WHEELBASE = 0.3240                             # distancia entre ejes [m]
MAX_STEER = 0.5236                             # direccion maxima [rad] = 30 deg
KAPPA_MAX = math.tan(MAX_STEER) / WHEELBASE    # curvatura maxima [1/m] ~ 1.78


# ------------------------------------------------------------------ rutas
def pkg_dir(name):
    """Carpeta hermana del modulo (maps/, waypoints/).

    Con `colcon build --symlink-install` este modulo se ejecuta desde build/,
    asi que la ruta relativa apuntaria ahi y los entregables no quedarian en el
    repositorio. Si se detecta que estamos bajo build/ o install/, se redirige
    al arbol de fuentes.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    for marker in ('/build/', '/install/'):
        if marker in here:
            root = here[:here.index(marker)]
            cand = os.path.join(root, 'src', 'global_planner', name)
            if os.path.isdir(os.path.dirname(cand)):
                os.makedirs(cand, exist_ok=True)
                return os.path.abspath(cand)
    return os.path.abspath(os.path.join(here, '..', name))


# ------------------------------------------------------------------ mapa
def load_map(yaml_path):
    """Devuelve (meta, occupied, free, unknown) del mapa trinario de SLAM."""
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)

    img = np.array(Image.open(img_path).convert('L')).astype(np.float64)

    # Convencion nav2: p = (255 - pixel)/255, salvo que negate = 1
    occ = img / 255.0 if int(meta.get('negate', 0)) else (255.0 - img) / 255.0

    occupied = occ >= float(meta.get('occupied_thresh', 0.65))
    free = occ <= float(meta.get('free_thresh', 0.196))
    unknown = ~occupied & ~free
    return meta, occupied, free, unknown


def find_track(free, seed=None):
    """Componente conexa de espacio libre que corresponde a la pista.

    Con `seed` (celda del spawn del vehiculo) se devuelve la componente que lo
    contiene. Es el criterio correcto: en un mapa de SLAM el area EXTERIOR al
    circuito tambien queda marcada como libre y normalmente es MAS GRANDE que
    la propia pista, de modo que "quedarse con la componente mayor" selecciona
    el exterior y ninguna ruta es posible. El vehiculo, en cambio, esta sobre
    la pista por definicion.

    Sin `seed` se cae al criterio de tamano (util para mapas sinteticos).
    """
    lbl, n = ndimage.label(free, structure=ndimage.generate_binary_structure(2, 2))
    if n == 0:
        raise RuntimeError('El mapa no tiene espacio libre. Revisa el .pgm/.yaml.')
    sizes = ndimage.sum(free, lbl, range(1, n + 1))

    if seed is None:
        return (lbl == (int(np.argmax(sizes)) + 1)), int(n), sizes

    H, W = free.shape
    r = int(np.clip(seed[0], 0, H - 1))
    c = int(np.clip(seed[1], 0, W - 1))
    k = int(lbl[r, c])
    if k == 0:
        # El spawn cayo sobre pared o zona desconocida: tomar la componente
        # libre mas cercana en vez de fallar.
        _, idx = ndimage.distance_transform_edt(lbl == 0, return_indices=True)
        k = int(lbl[idx[0][r, c], idx[1][r, c]])
    if k == 0:
        raise RuntimeError('No se encontro ninguna componente libre cerca del spawn.')
    return (lbl == k), int(n), sizes


def build_cost_field(track, resolution, clearance, penalty, d_ref):
    """Distancia a pared, mascara transitable y multiplicador de costo.

    dist     : distancia [m] de cada celda a la celda no-pista mas cercana
    drivable : celdas con holgura suficiente para el ancho del vehiculo
    mult     : >= 1, crece al acercarse a la pared -> Dijkstra prefiere el centro
    """
    dist = ndimage.distance_transform_edt(track) * float(resolution)
    drivable = dist >= float(clearance)
    prox = np.clip((float(d_ref) - dist) / float(d_ref), 0.0, 1.0)
    mult = 1.0 + float(penalty) * prox ** 2
    return dist, drivable, mult


# ------------------------------------------------------ coordenadas
def world_to_grid(x, y, meta, H):
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return H - 1 - int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res))


def grid_to_world(row, col, meta, H):
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return ox + (col + 0.5) * res, oy + (H - 1 - row + 0.5) * res


def path_to_world(path_px, meta, H):
    return np.array([grid_to_world(r, c, meta, H) for r, c in path_px])


def path_to_grid(xy, meta, H):
    return np.array([world_to_grid(x, y, meta, H) for x, y in xy])


def make_snapper(drivable):
    """Devuelve f(celda) -> celda transitable mas cercana."""
    _, idx = ndimage.distance_transform_edt(~drivable, return_indices=True)

    def snap(cell):
        r = int(np.clip(cell[0], 0, drivable.shape[0] - 1))
        c = int(np.clip(cell[1], 0, drivable.shape[1] - 1))
        if drivable[r, c]:
            return (r, c)
        return (int(idx[0][r, c]), int(idx[1][r, c]))

    return snap


# ------------------------------------------------------------------ Dijkstra
def dijkstra(drivable, mult, start, goal, record=False):
    """Dijkstra 8-conectado con w(a,b) = long(a,b) * (mult[a] + mult[b]) / 2.

    Todos los pesos son estrictamente positivos y no se usa heuristica alguna,
    asi que la optimalidad de Dijkstra se conserva: la ruta devuelta es la de
    minimo costo total bajo esa metrica (longitud penalizada por cercania a
    pared). El efecto practico del multiplicador es que la propia busqueda de
    "camino mas corto" prefiere el centro del corredor.

    Devuelve (celdas_de_la_ruta, orden_de_expansion, costo_total).
    """
    H, W = drivable.shape
    dist = np.full((H, W), np.inf)
    par_r = np.full((H, W), -1, dtype=np.int32)
    par_c = np.full((H, W), -1, dtype=np.int32)
    done = np.zeros((H, W), dtype=bool)
    order = []

    nbrs = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2))

    dist[start] = 0.0
    pq = [(0.0, start[0], start[1])]
    found = (start == goal)

    while pq:
        d, r, c = heapq.heappop(pq)
        if done[r, c]:
            continue
        done[r, c] = True
        if record:
            order.append((r, c))
        if (r, c) == goal:
            found = True
            break
        m_here = mult[r, c]
        for dr, dc, step in nbrs:
            nr, nc = r + dr, c + dc
            if nr < 0 or nc < 0 or nr >= H or nc >= W:
                continue
            if not drivable[nr, nc] or done[nr, nc]:
                continue
            if dr != 0 and dc != 0:                    # anti corner-cutting
                if not drivable[r, nc] or not drivable[nr, c]:
                    continue
            nd = d + step * 0.5 * (m_here + mult[nr, nc])
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                par_r[nr, nc] = r
                par_c[nr, nc] = c
                heapq.heappush(pq, (nd, nr, nc))

    if not found:
        raise RuntimeError(
            'Dijkstra no encontro ruta entre %s y %s. Prueba --clearance menor.'
            % (start, goal))

    path, cur = [goal], goal
    while cur != start:
        cur = (int(par_r[cur[0], cur[1]]), int(par_c[cur[0], cur[1]]))
        path.append(cur)
    path.reverse()
    return np.array(path, dtype=np.int32), order, float(dist[goal])


# ---------------------------------------------------------- vuelta completa
def lap_checkpoints(drivable, dist, spawn_cell, n_ckpt):
    """Reparte n_ckpt puntos de paso alrededor del anillo transitable.

    Dijkstra es punto-a-punto: en un circuito cerrado, planificar de inicio a
    meta con ambos en el mismo lugar devolveria una ruta nula. Se colocan
    checkpoints por angulo respecto al centroide de la pista, eligiendo dentro
    de cada sector la celda mas alejada de las paredes, y se planifica un tramo
    entre cada par consecutivo. El primero se ancla al spawn real del vehiculo.
    """
    rows, cols = np.nonzero(drivable)
    cr, cc = rows.mean(), cols.mean()

    # angulo en coordenadas de imagen (la fila crece hacia abajo -> se invierte)
    ang = np.arctan2(-(rows - cr), cols - cc)
    dvals = dist[rows, cols]
    theta0 = math.atan2(-(spawn_cell[0] - cr), spawn_cell[1] - cc)

    ckpts = [tuple(int(v) for v in spawn_cell)]
    for i in range(1, n_ckpt):
        target = theta0 + 2.0 * math.pi * i / n_ckpt
        diff = np.abs(np.arctan2(np.sin(ang - target), np.cos(ang - target)))
        tol = math.pi / n_ckpt * 0.35
        sel = diff < tol
        while not sel.any() and tol < math.pi:
            tol *= 1.6
            sel = diff < tol
        k = np.nonzero(sel)[0]
        best = k[int(np.argmax(dvals[k]))]          # el mas centrado del sector
        ckpts.append((int(rows[best]), int(cols[best])))
    return ckpts, (cr, cc)


def plan_lap(drivable, mult, ckpts, record=True, verbose=True):
    """Encadena tramos Dijkstra entre checkpoints consecutivos y cierra la vuelta."""
    segs = list(zip(ckpts, ckpts[1:] + [ckpts[0]]))
    full, order, cost = [], [], 0.0
    for i, (a, b) in enumerate(segs):
        p, o, c = dijkstra(drivable, mult, a, b, record=record)
        cost += c
        if record:
            order.extend(o)
        full.append(p if i == 0 else p[1:])          # no duplicar el empalme
        if verbose:
            print('    tramo %2d/%d: %4d celdas, costo %8.1f'
                  % (i + 1, len(segs), len(p), c))
    return np.vstack(full), order, cost


# ------------------------------------------------------------------ suavizado
def resample(xy, resolution, step_m):
    """Submuestrea la ruta cruda a un paso aproximado en metros."""
    k = max(1, int(round(step_m / resolution)))
    pts = xy[::k]
    if not np.allclose(pts[-1], xy[-1]):
        pts = np.vstack([pts, xy[-1]])
    keep = [0]
    for i in range(1, len(pts)):
        if np.linalg.norm(pts[i] - pts[keep[-1]]) > 1e-9:
            keep.append(i)
    return pts[keep]


def bspline(ctrl, s, closed, n_out=2000):
    """B-Spline cubica parametrica. Devuelve (puntos, yaw, curvatura)."""
    pts, per = ctrl.copy(), 0
    if closed:
        per = 1
        if not np.allclose(pts[0], pts[-1]):
            pts = np.vstack([pts, pts[0]])   # splprep con per=1 ignora el ultimo
    if len(pts) < 5:
        raise RuntimeError('Muy pocos puntos de control para una B-Spline cubica.')

    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=float(s), k=3, per=per)
    u = np.linspace(0.0, 1.0, n_out)
    x, y = splev(u, tck)
    dx, dy = splev(u, tck, der=1)
    ddx, ddy = splev(u, tck, der=2)

    den = np.power(dx * dx + dy * dy, 1.5)
    den[den < 1e-12] = 1e-12
    curv = np.abs(dx * ddy - dy * ddx) / den
    return np.column_stack([x, y]), np.arctan2(dy, dx), curv


def count_collisions(xy, drivable, meta, H):
    bad = 0
    for x, y in xy:
        r, c = world_to_grid(x, y, meta, H)
        if not (0 <= r < drivable.shape[0] and 0 <= c < drivable.shape[1]):
            bad += 1
        elif not drivable[r, c]:
            bad += 1
    return bad


def smooth_path_safe(raw_xy, resolution, drivable, meta, H, closed, step_m=0.30):
    """Suavizado con escalera de seguridad.

    El factor s de la B-Spline controla el compromiso: mas s da una curva mas
    suave y de menor curvatura, pero tambien mas libertad para "cortar camino" y
    rozar una pared en una curva cerrada. En vez de fijar un valor a dedo, se
    prueba una escalera de mas agresivo a mas conservador y se toma el PRIMERO
    que cumple a la vez:
      (1) cero puntos de la curva fuera de la zona transitable, y
      (2) kappa_max <= tan(30 deg)/L = 1.78 1/m (limite cinematico del F1TENTH).
    """
    ctrl = resample(raw_xy, resolution, step_m)
    n = len(ctrl)
    ladder = [3.0 * n, 1.5 * n, 0.8 * n, 0.4 * n, 0.2 * n, 0.1 * n, 0.05 * n, 0.0]

    last = None
    for s in ladder:
        try:
            xy, yaw, curv = bspline(ctrl, s, closed)
        except Exception as exc:
            print('    s = %8.2f -> fallo el ajuste (%s)' % (s, exc))
            continue
        col = count_collisions(xy, drivable, meta, H)
        kmax = float(np.max(curv))
        ok = (col == 0 and kmax <= KAPPA_MAX)
        print('    s = %8.2f -> colisiones = %4d | kappa_max = %6.3f 1/m | %s'
              % (s, col, kmax, 'ACEPTADO' if ok else 'descartado'))
        last = (xy, yaw, curv, s, col, kmax)
        if ok:
            return last
    if last is None:
        raise RuntimeError('Ningun valor de suavizado produjo una curva valida.')
    print('    [!] Ningun s cumple ambas condiciones; se usa el ultimo evaluado.')
    return last


def path_length(xy):
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


# ------------------------------------------------------------------ dibujo
def canvas(occupied, drivable, track):
    """Fondo RGB: pista blanca, margen de seguridad gris, pared negra."""
    rgb = np.ones(occupied.shape + (3,)) * 0.93
    rgb[track] = [1.0, 1.0, 1.0]
    rgb[track & ~drivable] = [0.80, 0.83, 0.90]
    rgb[occupied] = [0.08, 0.08, 0.10]
    return rgb
