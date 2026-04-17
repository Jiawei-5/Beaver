# -*- coding: mbcs -*-
# Beaver.py  (Abaqus/CAE Python 2.7, Windows)
# Pipeline: STL -> shift / optional uniform scaling (<= MAX_SHRINK) -> merge -> mesh -> EasyPBC -> copy ODB -> stiffness extraction
# Notes: Put stlImport and 3DMesh_to_Geometry *.pyc modules in the same folder as this script (auto-detected).

from __future__ import print_function
from abaqus import *
from abaqusConstants import *
from driverUtils import executeOnCaeStartup
executeOnCaeStartup()
from caeModules import *
import regionToolset, mesh

import os, sys, csv, time, glob, shutil, traceback, datetime, re, math
import numpy as np
try:
    import Tkinter as tk
    import ttk, tkFileDialog, tkMessageBox
except:
    tk = None

# ---------------- Strings / Viewport ----------------
def nstr(s):
    try:
        if isinstance(s, str): return s
        return s.encode('mbcs')
    except:
        return str(s)

def clear_viewports():
    try:
        for vp in session.viewports.values():
            vp.setValues(displayedObject=None)
    except:
        pass

# ---------------- Logging / Exit ----------------
ORIG_STDOUT, ORIG_STDERR, LOG_FH = sys.stdout, sys.stderr, None

class Tee(object):
    def __init__(self, *streams):
        self.streams = streams
    def write(self, data):
        for s in self.streams:
            try:
                s.write(data)
            except:
                pass
    def flush(self):
        for s in self.streams:
            try:
                s.flush()
            except:
                pass

def _ts():
    return datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

def init_logging(work_dir, fname=None):
    global ORIG_STDOUT, ORIG_STDERR, LOG_FH
    try:
        log_dir = os.path.join(nstr(work_dir), 'logs')
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        if fname is None:
            stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = 'Beaver_%s_%d.log' % (stamp, os.getpid())
        log_path = os.path.join(log_dir, fname)
        if LOG_FH is None:
            LOG_FH = open(log_path, 'ab')
            sys.stdout = Tee(ORIG_STDOUT, LOG_FH)
            sys.stderr = Tee(ORIG_STDERR, LOG_FH)
            print('='*72)
            print('[%s] Log started: %s' % (_ts(), log_path))
            print('PID=%d  Python=%s  Abaqus-CAE' % (os.getpid(), sys.version.split()[0]))
            print('='*72)
        return log_path
    except Exception:
        sys.stdout = ORIG_STDOUT
        sys.stderr = ORIG_STDERR
        return None

def _close_all_odbs():
    try:
        for nm, odb in session.odbs.items():
            try:
                odb.close()
            except:
                pass
    except:
        pass

def end_logging():
    """Restore stdout/stderr and safely close the log file."""
    global ORIG_STDOUT, ORIG_STDERR, LOG_FH
    try:
        if LOG_FH is not None:
            try:
                sys.stdout.flush()
                sys.stderr.flush()
                LOG_FH.flush()
            except:
                pass
            try:
                LOG_FH.close()
            except:
                pass
    finally:
        LOG_FH = None
        # Restore original stdout/stderr (avoid downstream print errors)
        sys.stdout = ORIG_STDOUT
        sys.stderr = ORIG_STDERR

def safe_exit(exit_code=0):
    """
    Terminate the current script:
      - close all opened ODBs
      - flush and close the log
      - raise SystemExit so Abaqus / command line exits cleanly
    Avoid forcing os._exit to prevent killing the whole CAE session.
    """
    try:
        _close_all_odbs()
    except:
        pass
    try:
        end_logging()
    except:
        pass
    # Exit the current script / Python interpreter normally
    raise SystemExit(exit_code)
    # Equivalent alternative:
    # sys.exit(exit_code)
# ---------------- Default parameters ----------------
DEFAULTS = dict(
    WORK_DIR            = r'D:',
    SAVE_CAE_NAME       = 'default.cae',
    STL_DIR             = r'D:',
    EASYPBC_CPU         = 14,
    EASYPBC_MESHSENS    = 1e-7,
    Lx=4.0, Ly=4.0, Lz=4.0,
    MESH_SIZE=0.20,
    MAT_MATRIX_E=1000.0, MAT_MATRIX_NU=0.3,
    MAT_FIBRE_E=3000.0, MAT_FIBRE_NU=0.4,
    AUTO_EXIT=False,
    MAX_SHRINK=0.00     # Maximum allowed uniform shrink (0.05 = 5%)
)

CASES = ['E11','E22','E33','G23','G13','G12']

APP_NAME = 'Beaver'
RUN_ROOT_DIRNAME = '_beaver_run'

# ---------------- Post-processing settings ----------------
# Set to False if you want a quieter log during ODB-based stiffness extraction.
PRINT_POSTPROC_INFO = True

# Preferred strain field for volume-averaged strain extraction.
# Use 'E' for engineering/small strain. If unavailable, Beaver can fall back to 'LE'.
STRAIN_FIELD = 'E'
STRAIN_ALLOW_FALLBACK = True

# Abaqus E/LE shear components are usually engineering shear strains
# (gamma12, gamma13, gamma23). Leave this as True for standard solid analyses.
SHEAR_IS_ENGINEERING = True

# Optional window icon file placed next to this script.
# Supported: .ico (best on Windows), .png, .jpg (jpg requires Pillow in some environments).
# Tk window icon (optional).
# Put your own icon image next to Beaver.py and set CUSTOM_TK_ICON below.
# Recommended size: 32x32 or 64x64.
# Supported (best -> worst on Windows): .ico, .png, .gif, .jpg/.jpeg (jpg requires Pillow).
CUSTOM_TK_ICON = ''  # Example: 'my_beaver.ico' or 'my_beaver.png' (leave empty for auto-detect)
ICON_CANDIDATES = [
    'Beaver.ico',
    'Beaver.png',
    'Beaver.jpg',
    'Beaver.jpeg',
    'Beaver.gif',
]

# Resolve this script folder robustly.
# Abaqus may run scripts via execfile() where __file__ is not defined.
def _resolve_this_dir():
    """Return the folder containing Beaver.py even if __file__ is undefined."""
    # 1) Normal module import
    try:
        f = globals().get('__file__', '')
        if f:
            return os.path.dirname(os.path.abspath(f))
    except:
        pass

    # 2) Inspect the current code object filename (works for execfile)
    try:
        import inspect
        f = inspect.getsourcefile(lambda: 0)
        if not f:
            f = inspect.getfile(inspect.currentframe())
        if f and (f not in ('<string>', '<stdin>')):
            f = os.path.abspath(f)
            if os.path.exists(f):
                return os.path.dirname(f)
    except:
        pass

    # 3) Search sys.path for Beaver.py (plug-in folders are typically mounted there)
    try:
        for p in list(sys.path):
            try:
                cand = os.path.join(p, 'Beaver.py')
                if os.path.isfile(cand):
                    return os.path.dirname(os.path.abspath(cand))
            except:
                pass
    except:
        pass

    # 4) Fallback: current working directory
    try:
        return os.getcwd()
    except:
        return '.'

THIS_DIR = _resolve_this_dir()

# Only the runtime-imported modules are mandatory. Their helper modules are
# resolved automatically if they live in the same folder as the main module.
RUNTIME_REQUIRED_MODULES = ('stl2inp', 'mesh_geo')


# ---------------- General utilities ----------------
def ensure_path(p):
    p = nstr(p)
    if p and (p not in sys.path):
        sys.path.insert(0, p)



def _dependency_search_dirs():
    """Return candidate directories for stlImport / 3DMesh_to_Geometry modules.

    Search order:
      1) Beaver/_deps and other local subfolders
      2) Beaver folder itself
      3) sibling folders under the same abaqus_plugins directory, such as
         stlImport and 3DMesh_to_Geometry

    This lets Beaver run without bundling duplicate *.pyc files when the host
    machine already has those plug-ins installed.
    """
    base = THIS_DIR
    parent = os.path.dirname(base)

    raw = [
        os.path.join(base, '_deps'),
        os.path.join(base, 'deps'),
        os.path.join(base, 'vendor'),
        os.path.join(base, 'plugins'),
        base,
        os.path.join(parent, 'stlImport'),
        os.path.join(parent, '3DMesh_to_Geometry'),
    ]

    out = []
    seen = set()
    for d in raw:
        try:
            d = os.path.abspath(nstr(d))
        except:
            d = nstr(d)
        key = d.lower()
        if key in seen:
            continue
        seen.add(key)
        if os.path.isdir(d):
            out.append(d)
    return out



def _mount_dependency_dirs():
    dirs = _dependency_search_dirs()
    added = []
    for d in reversed(dirs):
        if d not in sys.path:
            sys.path.insert(0, d)
            added.append(d)
    return dirs, added



def _import_module_probe(name):
    try:
        __import__(name)
        return True, ''
    except Exception as e:
        return False, str(e)



def ensure_runtime_dependencies(strict=False, verbose=False):
    """Make Beaver dependencies importable.

    strict=False:
        silently mount candidate paths; do not fail CAE startup.
    strict=True:
        require stl2inp and mesh_geo to be importable, otherwise raise a clear error.

    Beaver no longer requires duplicate local copies of all *.pyc files. If the
    official sibling folders already exist, Beaver can reuse them directly.
    """
    dirs, added = _mount_dependency_dirs()

    missing = []
    errors = {}
    for mod in RUNTIME_REQUIRED_MODULES:
        ok, err = _import_module_probe(mod)
        if not ok:
            missing.append(mod)
            errors[mod] = err

    if strict and missing:
        msg = []
        msg.append('[%s] Required runtime modules could not be imported:' % APP_NAME)
        for mod in missing:
            if errors.get(mod):
                msg.append('  - %s  (%s)' % (mod, errors[mod]))
            else:
                msg.append('  - %s' % mod)
        msg.append('')
        msg.append('Searched directories:')
        if dirs:
            for d in dirs:
                msg.append('  - %s' % d)
        else:
            msg.append('  - (no candidate directory found)')
        msg.append('')
        msg.append('Fix one of these layouts:')
        msg.append('  1) Keep the official sibling folders: stlImport and 3DMesh_to_Geometry')
        msg.append('  2) Or place their *.pyc files inside Beaver/_deps')
        raise RuntimeError('\n'.join(msg))

    if verbose and dirs:
        print('[%s] Dependency search directories:' % APP_NAME)
        for d in dirs:
            print('  - %s' % d)

    return dirs, added


# Mount candidate dependency folders at import time, but do not error here.
try:
    ensure_runtime_dependencies(strict=False, verbose=False)
except Exception:
    pass


def list_stl_files(stl_dir):
    stl_dir = nstr(stl_dir)
    if not os.path.isdir(stl_dir):
        raise IOError('STL directory does not exist: %s' % stl_dir)
    files = [os.path.join(stl_dir, f) for f in os.listdir(stl_dir) if f.lower().endswith('.stl')]
    files.sort()
    return files

def model_name_from_file(fp):
    base = os.path.splitext(os.path.basename(fp))[0]
    name = ''.join([c if (c.isalnum() or c in '._-') else '_' for c in base])
    if name and name[0].isdigit(): name = 'M_' + name
    return name

# ---------------- STL -> orphan mesh (with tolerance retry) ----------------
def import_stl_with_retry(stl_path, model_name, tries=None):
    """Import STL using the local stlImport plug-in (stl2inp) with tolerance retry."""
    ensure_runtime_dependencies(strict=True)
    import stl2inp
    if tries is None:
        tries = [1e-6, 1e-5, 1e-4, 1e-3]

    if nstr(model_name) in mdb.models:
        del mdb.models[nstr(model_name)]
    mdb.Model(name=nstr(model_name))

    for tol in tries:
        try:
            clear_viewports()
            stl2inp.STL2inp(stlfile=nstr(stl_path),
                            modelName=nstr(model_name),
                            mergeNodesTolerance=tol)
            print('  STL2inp OK @ tol =', tol)
            return True
        except Exception as e:
            print('  STL2inp failed @ tol = %g : %s' % (tol, str(e)))
    return False

def _inst_geom_bbox(inst):
    xs, ys, zs = [], [], []
    for v in inst.vertices:
        x, y, z = v.pointOn[0]; xs.append(x); ys.append(y); zs.append(z)
    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

def _fit_shift_axis(vmin, vmax, L, margin):
    half  = 0.5*(vmax - vmin)           # Half-size
    allow = 0.5*L - margin              # Allowed half-size
    if half > allow + 1e-12:            # Oversized: scaling is required
        return False, 0.0
    ccur  = 0.5*(vmin + vmax)           # Current center
    low   = -allow + half
    high  =  allow - half
    ctar  = min(max(ccur, low), high)   # Clamp to feasible interval
    return True, (ctar - ccur)

def _part_geom_bbox(partObj):
    xs, ys, zs = [], [], []
    for v in partObj.vertices:
        x, y, z = v.pointOn[0]; xs.append(x); ys.append(y); zs.append(z)
    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

def _seed_outer_edges_structured_faces(pM, bbox, mesh_size, Nx=None, Ny=None, Nz=None):
    xmin,xmax, ymin,ymax, zmin,zmax = bbox
    Lx=max(xmax-xmin,1e-12); Ly=max(ymax-ymin,1e-12); Lz=max(zmax-zmin,1e-12)
    if Nx is None: Nx=max(2, int(round(Lx/max(mesh_size,1e-9))))
    if Ny is None: Ny=max(2, int(round(Ly/max(mesh_size,1e-9))))
    if Nz is None: Nz=max(2, int(round(Lz/max(mesh_size,1e-9))))
    eps = 1e-6*max(Lx,Ly,Lz)
    edges=pM.edges
    left_edges  =tuple(edges.getByBoundingBox(xMin=xmin-eps, xMax=xmin+eps))
    right_edges =tuple(edges.getByBoundingBox(xMin=xmax-eps, xMax=xmax+eps))
    bot_edges   =tuple(edges.getByBoundingBox(yMin=ymin-eps, yMax=ymin+eps))
    top_edges   =tuple(edges.getByBoundingBox(yMin=ymax-eps, yMax=ymax+eps))
    back_edges  =tuple(edges.getByBoundingBox(zMin=zmin-eps, zMax=zmin+eps))
    front_edges =tuple(edges.getByBoundingBox(zMin=zmax-eps, zMax=zmax+eps))
    if left_edges:  pM.seedEdgeByNumber(edges=left_edges,  number=max(Ny,Nz), constraint=FINER)
    if right_edges: pM.seedEdgeByNumber(edges=right_edges, number=max(Ny,Nz), constraint=FINER)
    if bot_edges:   pM.seedEdgeByNumber(edges=bot_edges,   number=max(Nz,Nx), constraint=FINER)
    if top_edges:   pM.seedEdgeByNumber(edges=top_edges,   number=max(Nz,Nx), constraint=FINER)
    if back_edges:  pM.seedEdgeByNumber(edges=back_edges,  number=max(Nx,Ny), constraint=FINER)
    if front_edges: pM.seedEdgeByNumber(edges=front_edges, number=max(Nx,Ny), constraint=FINER)
    # Try to enforce STRUCTURED meshing on the 6 outer faces (if geometry allows)
    faces=pM.faces
    try:
        fL=tuple(faces.getByBoundingBox(xMin=xmin-eps, xMax=xmin+eps))
        fR=tuple(faces.getByBoundingBox(xMin=xmax-eps, xMax=xmax+eps))
        fB=tuple(faces.getByBoundingBox(yMin=ymin-eps, yMax=ymin+eps))
        fT=tuple(faces.getByBoundingBox(yMin=ymax-eps, yMax=ymax+eps))
        fK=tuple(faces.getByBoundingBox(zMin=zmin-eps, zMax=zmin+eps))
        fF=tuple(faces.getByBoundingBox(zMin=zmax-eps, zMax=zmax+eps))
        for fcs in (fL,fR,fB,fT,fK,fF):
            if fcs:
                try: pM.setMeshControls(regions=fcs, technique=STRUCTURED)
                except: pass
    except: pass

def build_models(params):
    """Build one Abaqus model per STL file: STL -> solid geometry -> merge into a cube -> mesh."""
    stl_list = list_stl_files(params['STL_DIR'])
    if not stl_list:
        print('[WARN] No *.stl files found in:', params['STL_DIR'])
        return []

    ensure_runtime_dependencies(strict=True)
    import mesh_geo

    Lx = float(params['Lx']); Ly = float(params['Ly']); Lz = float(params['Lz'])
    mesh_size = float(params['MESH_SIZE'])
    margin = 0.5 * mesh_size
    max_shrink = float(params.get('MAX_SHRINK', 0.0))

    # Record scaling / skip status
    fit_csv = os.path.join(nstr(params['WORK_DIR']), 'fit_report.csv')
    if not os.path.isdir(nstr(params['WORK_DIR'])):
        os.makedirs(nstr(params['WORK_DIR']))
    if not os.path.exists(fit_csv):
        with open(fit_csv, 'wb') as f:
            w = csv.writer(f)
            w.writerow(['Model', 'Status', 'ShrinkUsed', 'V_before', 'V_after'])

    built = []
    for fp in stl_list:
        tag = model_name_from_file(fp)
        print('\n=== BUILD MODEL: %s  (from %s) ===' % (tag, fp))

        # 1) STL -> PART-1 (orphan mesh)
        if not import_stl_with_retry(fp, tag):
            print('  [Skip] STL import failed.')
            continue

        mdl = mdb.models[nstr(tag)]

        # 2) Orphan mesh -> solid geometry 'GRAIN'
        clear_viewports()
        mesh_geo.Run(modelName=nstr(tag),
                     meshPartName=nstr('PART-1'),
                     geoPartName=nstr('GRAIN'),
                     solid=True)
        if nstr('PART-1') in mdl.parts:
            del mdl.parts[nstr('PART-1')]

        # 3) Build cube C4 (0..L -> centered)
        s = mdl.ConstrainedSketch(name=nstr('__cube__'), sheetSize=10.0 * max(Lx, Ly))
        s.rectangle(point1=(0.0, 0.0), point2=(Lx, Ly))
        pC = mdl.Part(name=nstr('C4'), dimensionality=THREE_D, type=DEFORMABLE_BODY)
        pC.BaseSolidExtrude(sketch=s, depth=Lz)
        del mdl.sketches[nstr('__cube__')]

        # 4) Materials and sections (density not required for stiffness extraction)
        if nstr('matrix') not in mdl.materials:
            matM = mdl.Material(name=nstr('matrix'))
            matM.Elastic(table=((params['MAT_MATRIX_E'], params['MAT_MATRIX_NU']),))
            mdl.HomogeneousSolidSection(name=nstr('matrix'), material=nstr('matrix'))

        if nstr('fibre') not in mdl.materials:
            matF = mdl.Material(name=nstr('fibre'))
            matF.Elastic(table=((params['MAT_FIBRE_E'], params['MAT_FIBRE_NU']),))
            mdl.HomogeneousSolidSection(name=nstr('fibre'), material=nstr('fibre'))

        pG = mdl.parts[nstr('GRAIN')]
        pC.SectionAssignment(region=regionToolset.Region(cells=pC.cells), sectionName=nstr('matrix'))
        pG.SectionAssignment(region=regionToolset.Region(cells=pG.cells), sectionName=nstr('fibre'))

        # 5) Assembly: center cube; auto shift / conditional scaling for GRAIN
        a = mdl.rootAssembly
        a.DatumCsysByDefault(CARTESIAN)
        instC = a.Instance(name=nstr('C4-1'), part=pC, dependent=ON)
        instG = a.Instance(name=nstr('GRAIN-1'), part=pG, dependent=ON)

        a.translate(instanceList=(nstr('C4-1'),), vector=(-Lx/2.0, -Ly/2.0, -Lz/2.0))

        # Check whether GRAIN needs shifting (and/or scaling)
        gxmin, gxmax, gymin, gymax, gzmin, gzmax = _inst_geom_bbox(instG)
        okx, dx = _fit_shift_axis(gxmin, gxmax, Lx, margin)
        oky, dy = _fit_shift_axis(gymin, gymax, Ly, margin)
        okz, dz = _fit_shift_axis(gzmin, gzmax, Lz, margin)

        def _write_fit_row(status, sk=0.0, v0='', v1=''):
            with open(fit_csv, 'ab') as f:
                w = csv.writer(f)
                w.writerow([tag, status, '%.6f' % sk, v0, v1])

        if okx and oky and okz:
            if abs(dx) > 1e-12 or abs(dy) > 1e-12 or abs(dz) > 1e-12:
                a.translate(instanceList=(nstr('GRAIN-1'),), vector=(dx, dy, dz))
            fit_status = 'shift_only'
            _write_fit_row(fit_status, 0.0, '', '')
        else:
            # Scaling required: compute minimum uniform scale factor s_req (<= 1.0)
            halfx = 0.5 * (gxmax - gxmin)
            halfy = 0.5 * (gymax - gymin)
            halfz = 0.5 * (gzmax - gzmin)
            allowx = 0.5 * Lx - margin
            allowy = 0.5 * Ly - margin
            allowz = 0.5 * Lz - margin
            s_req = min(allowx/halfx if halfx > 0 else 1.0,
                        allowy/halfy if halfy > 0 else 1.0,
                        allowz/halfz if halfz > 0 else 1.0)
            s_req = min(1.0, max(0.0, s_req))
            shrink = 1.0 - s_req

            if (s_req < 1.0) and (shrink <= max_shrink + 1e-12):
                # Record volume
                try:
                    V0 = pG.getMassProperties()['volume']
                except:
                    V0 = ''

                # Scale the instance (keep a small clearance margin)
                try:
                    a.Scale(instanceList=(nstr('GRAIN-1'),), scale=s_req * 0.999)
                except:
                    # Some Abaqus versions use lowercase API name
                    try:
                        a.scale(instanceList=(nstr('GRAIN-1'),), scale=s_req * 0.999)
                    except:
                        pass

                a.regenerate()

                # Shift again after scaling
                gxmin, gxmax, gymin, gymax, gzmin, gzmax = _inst_geom_bbox(instG)
                okx, dx = _fit_shift_axis(gxmin, gxmax, Lx, margin)
                oky, dy = _fit_shift_axis(gymin, gymax, Ly, margin)
                okz, dz = _fit_shift_axis(gzmin, gzmax, Lz, margin)

                if okx and oky and okz:
                    a.translate(instanceList=(nstr('GRAIN-1'),), vector=(dx, dy, dz))
                    V1 = (V0 * (s_req**3)) if (V0 != '') else ''
                    fit_status = 'scaled'
                    _write_fit_row(fit_status, shrink,
                                   '%.6g' % V0 if V0 != '' else '',
                                   '%.6g' % V1 if V1 != '' else '')
                else:
                    print('  [Skip] Still outside cube after scaling:', tag)
                    _write_fit_row('skip_oversize_after_scale', shrink,
                                   '%.6g' % V0 if V0 != '' else '', '')
                    continue
            else:
                print('  [Skip] Required shrink %.2f%% exceeds limit %.2f%%: %s'
                      % (shrink*100.0, max_shrink*100.0, tag))
                _write_fit_row('skip_oversize', shrink, '', '')
                continue

        a.regenerate()

        # 6) Boolean merge MERGE_<tag> and instantiate immediately
        merge_name = 'MERGE_%s' % tag
        a.InstanceFromBooleanMerge(name=nstr(merge_name),
                                   instances=(instC, instG),
                                   keepIntersections=ON,
                                   originalInstances=SUPPRESS,
                                   mergeNodes=NONE,
                                   domain=BOTH)
        if nstr(merge_name) in mdl.parts:
            inst_merge = nstr(merge_name + '-1')
            if inst_merge not in a.instances.keys():
                a.Instance(name=inst_merge, part=mdl.parts[nstr(merge_name)], dependent=ON)
        a.regenerate()

        # 7) Mesh: seed outer edges / attempt structured outer faces -> volume mesh
        pM = mdl.parts[nstr(merge_name)]
        pM.seedPart(size=mesh_size, deviationFactor=0.1, minSizeFactor=0.1)
        pM.setMeshControls(regions=pM.cells, elemShape=TET, technique=FREE)
        et1 = mesh.ElemType(elemCode=C3D20RH, elemLibrary=STANDARD)
        et2 = mesh.ElemType(elemCode=C3D15H, elemLibrary=STANDARD)
        et3 = mesh.ElemType(elemCode=C3D10H, elemLibrary=STANDARD, secondOrderAccuracy=OFF)
        pM.setElementType(regions=(pM.cells,), elemTypes=(et1, et2, et3))
        bbox = _part_geom_bbox(pM)
        _seed_outer_edges_structured_faces(pM, bbox, mesh_size)
        pM.generateMesh()
        a.regenerate()

        built.append(tag)
        print('  OK -> Model:', tag)

    # 8) Save CAE
    if not os.path.isdir(nstr(params['WORK_DIR'])):
        os.makedirs(nstr(params['WORK_DIR']))
    save_path = os.path.join(params['WORK_DIR'], params['SAVE_CAE_NAME'])
    mdb.saveAs(pathName=nstr(save_path))
    print('\n[CAE saved] ->', save_path)

    return built

def find_merge_instance_name(model_name):
    """Return the merged instance name (MERGE_*) in the given model."""
    mdl = mdb.models[nstr(model_name)]
    a = mdl.rootAssembly

    # If there is no instance yet, try to instantiate the MERGE_* part automatically.
    if len(a.instances.keys()) == 0:
        for pnm in mdl.parts.keys():
            if str(pnm).upper().startswith('MERGE_'):
                instnm = nstr(str(pnm) + '-1')
                a.Instance(name=instnm, part=mdl.parts[pnm], dependent=ON)
                a.regenerate()
                return instnm

        # Fallback: single-part model
        if len(mdl.parts.keys()) == 1:
            pnm = mdl.parts.keys()[0]
            instnm = nstr(str(pnm) + '-1')
            a.Instance(name=instnm, part=mdl.parts[pnm], dependent=ON)
            a.regenerate()
            return instnm

        raise RuntimeError(
            "Start-up error 03: Cannot locate a merged instance in model '%s'. "
            "Instances: %s, Available parts: %s"
            % (model_name, list(a.instances.keys()), list(mdl.parts.keys()))
        )

    # Prefer instances whose partName starts with MERGE_
    for inst_name, ins in a.instances.items():
        try:
            pn = getattr(ins, 'partName', '')
            if isinstance(pn, str) and pn.upper().startswith('MERGE_'):
                return inst_name
        except:
            pass

    # Heuristic: name contains 'merge'
    for inst_name in a.instances.keys():
        if 'merge' in inst_name.lower():
            return inst_name

    # Fallback: single instance
    if len(a.instances.keys()) == 1:
        return a.instances.keys()[0]

    raise RuntimeError(
        "Start-up error 03: Cannot locate a merged instance in model '%s'. Instances: %s"
        % (model_name, list(a.instances.keys()))
    )

def cleanup_assembly_keep_only(mdl, keep_inst_name):
    a=mdl.rootAssembly; clear_viewports()
    others=[nm for nm in a.instances.keys() if nm!=keep_inst_name]
    for nm in others:
        try: a.deleteInstances((nstr(nm),))
        except: pass
    feat=[]
    for fname,ft in a.features.items():
        try:
            if fname!=keep_inst_name and getattr(ft,'isSuppressed',False): feat.append(fname)
        except: pass
    for nm in feat:
        try: a.deleteFeatures((nstr(nm),))
        except: pass
    clear_viewports()

def chdir_workdir(work_dir):
    wd=nstr(work_dir)
    if not os.path.isdir(wd): os.makedirs(wd)
    os.chdir(wd)

def purge_job_handles():
    for case in CASES:
        nm=nstr('job-'+case)
        try:
            if nm in mdb.jobs.keys(): del mdb.jobs[nm]
        except: pass

def _safe_remove_file(path, retries=30, delay=0.5):
    """Robust file removal on Windows (handles transient locks / delays)."""
    path = nstr(path)
    if not os.path.exists(path):
        return True
    for _ in range(retries):
        try:
            os.remove(path)
            return True
        except:
            time.sleep(delay)
    # Last resort: try renaming so it will not block subsequent runs.
    try:
        stale = path + '.stale_%d' % int(time.time())
        os.rename(path, stale)
        return True
    except:
        return False


def _safe_rmtree(path, retries=30, delay=0.5):
    """Robust directory removal."""
    path = nstr(path)
    if not os.path.isdir(path):
        return True
    for _ in range(retries):
        try:
            shutil.rmtree(path)
            return True
        except:
            time.sleep(delay)
    return False


def _make_run_dir(run_root, tag):
    """Create a unique per-model working folder to avoid stale job-*.lck collisions."""
    run_root = nstr(run_root)
    if not os.path.isdir(run_root):
        os.makedirs(run_root)

    stamp = time.strftime('%Y%m%d_%H%M%S')
    base = '%s_%s_%d' % (tag, stamp, os.getpid())
    base = ''.join([c if (c.isalnum() or c in '._-') else '_' for c in base])
    d = os.path.join(run_root, base)
    os.makedirs(d)
    return d


def purge_job_files(work_dir, verbose=False):
    """Delete job-*.* files under the given folder (with retries)."""
    work_dir = nstr(work_dir)
    for fp in glob.glob(os.path.join(work_dir, 'job-*.*')):
        ok = _safe_remove_file(fp)
        if (not ok) and verbose:
            print('  [Warn] Could not remove:', fp)

def wait_odb_stable(work_dir, case, start_time, timeout=36000, interval=2):
    name='job-%s.odb'%case; last=-1; stable=0; t0=time.time()
    while True:
        p=os.path.join(nstr(work_dir),name)
        if os.path.exists(p) and os.path.getmtime(p)>=start_time:
            sz=os.path.getsize(p)
            if sz==last: stable+=1
            else: stable=0; last=sz
            if stable>=3: return p
        time.sleep(interval)
        if time.time()-t0>timeout: raise RuntimeError('Timeout waiting for %s'%name)

def copy_odb_safe(src,dst,tries=120,interval=5):
    src=nstr(src); dst=nstr(dst)
    for _ in range(tries):
        try:
            if os.path.exists(dst): os.remove(dst)
            shutil.copy2(src,dst); return
        except: time.sleep(interval)
    raise RuntimeError('Copy ODB failed: %s -> %s'%(src,dst))

def run_easypbc_for_models(params, model_names=None):
    """Run EasyPBC for each model and copy job-*.odb to <Model>_<Case>.odb in WORK_DIR."""
    work_dir_main = nstr(params['WORK_DIR'])
    chdir_workdir(work_dir_main)

    cpu = int(params.get('EASYPBC_CPU', 14))
    meshsens = float(params.get('EASYPBC_MESHSENS', 1e-7))

    if model_names is None:
        model_names = [m for m in mdb.models.keys() if m != 'Model-1']

    run_root = os.path.join(work_dir_main, RUN_ROOT_DIRNAME)
    if not os.path.isdir(run_root):
        os.makedirs(run_root)

    for tag in model_names:
        print('\n=== EasyPBC on: %s ===' % tag)

        if nstr(tag) not in mdb.models:
            print('  [Skip] Model not found:', tag)
            continue

        mdl = mdb.models[nstr(tag)]
        try:
            inst = find_merge_instance_name(tag)
            print('  Using instance:', inst)
        except Exception as e:
            print('  [Error]', str(e))
            continue

        # Clean assembly (keep only the merged instance)
        try:
            cleanup_assembly_keep_only(mdl, inst)
            print('  Assembly cleaned; keep only:', inst)
        except Exception as e:
            print('  [Warn] Assembly cleanup failed:', str(e))

        # Clear job objects and close any open ODBs before running
        purge_job_handles()
        _close_all_odbs()

        # Run each model in its own folder to avoid stale job-*.lck collisions
        run_dir = _make_run_dir(run_root, tag)
        main_cwd = os.getcwd()
        print('  Run directory:', run_dir)

        try:
            os.chdir(run_dir)
            purge_job_files(run_dir)

            clear_viewports()
            run_t0 = time.time()

            feasypbc(part=nstr(tag),
                     inst=nstr(inst),
                     meshsens=meshsens,
                     CPU=cpu,
                     E11=True, E22=True, E33=True,
                     G12=True, G13=True, G23=True,
                     onlyPBC=False,
                     CTE=False,
                     intemp=0,
                     fntemp=100,
                     reusePBC=True,
                     requestIVOL=True)

            print('  EasyPBC submitted.')

            # Copy ODBs back to main folder with tag prefix
            for case in CASES:
                src = wait_odb_stable(run_dir, case, start_time=run_t0)
                dst = os.path.join(work_dir_main, '%s_%s.odb' % (tag, case))
                copy_odb_safe(src, dst)
                print('    Saved:', dst)

        except Exception as e:
            print('  [Error] EasyPBC or ODB copy failed:', str(e))
            print(traceback.format_exc())
            # Keep run_dir for debugging
            try:
                os.chdir(main_cwd)
            except:
                pass
            continue

        finally:
            try:
                os.chdir(main_cwd)
            except:
                pass

        # Close ODBs to release file handles, then clean temp folder
        _close_all_odbs()
        purge_job_handles()

        try:
            purge_job_files(run_dir)
            if not _safe_rmtree(run_dir):
                print('  [Warn] Could not remove run directory (may be locked):', run_dir)
        except Exception as e:
            print('  [Warn] Temp cleanup failed:', str(e))

        print('  Done:', tag)

def _repo_has(repo, key):
    """Compatibility helper for Abaqus repositories: has_key / in."""
    try:
        return repo.has_key(key)  # noqa (Py2)
    except Exception:
        try:
            return key in repo.keys()
        except Exception:
            return False


# ----------------- Robust 'last frame' selection -----------------
def _step_sort_key(name):
    s = str(name)
    # Common step names: Step-1, Step-2, ...
    if '-' in s:
        tail = s.split('-')[-1]
        try:
            return int(tail)
        except Exception:
            pass
    return 10**9  # Put non-numeric suffixes at the end


def _pick_last_frame(odb, odb_path=''):
    step_names = list(odb.steps.keys())
    if not step_names:
        raise ValueError('ODB has no steps: %s' % odb_path)

    step_names_sorted = sorted(step_names, key=_step_sort_key)

    # Search backwards: the last step that has at least one frame
    for sn in reversed(step_names_sorted):
        st = odb.steps[sn]
        if st.frames and len(st.frames) > 0:
            return sn, st.frames[-1]

    raise ValueError('No frames in any step: %s' % odb_path)


# ----------------- Key construction (include sectionPoint to avoid shell/beam mismatch) -----------------
def _section_point_key(sp):
    if sp is None:
        return None
    for attr in ('number', 'label', 'index'):
        if hasattr(sp, attr):
            try:
                return int(getattr(sp, attr))
            except Exception:
                return getattr(sp, attr)
    # fallback
    try:
        return int(sp)
    except Exception:
        return repr(sp)


def _ip_key(v):
    inst = ''
    try:
        inst = v.instance.name
    except Exception:
        inst = ''
    el = int(v.elementLabel)
    ip = getattr(v, 'integrationPoint', 1)
    sp = getattr(v, 'sectionPoint', None)
    return (inst, el, ip, _section_point_key(sp))


def _elem_key(v):
    inst = ''
    try:
        inst = v.instance.name
    except Exception:
        inst = ''
    el = int(v.elementLabel)
    sp = getattr(v, 'sectionPoint', None)
    return (inst, el, _section_point_key(sp))


# ----------------- Components -> Voigt(6): [11,22,33,23,13,12] -----------------
def _voigt6_from_field_value(field_output, field_value):
    """
    Use componentLabels for a robust mapping (2D/3D compatible).
    Target order: [11, 22, 33, 23, 13, 12]
    """
    data = field_value.data
    try:
        _ = len(data)
    except Exception:
        data = (data,)

    labels = getattr(field_output, 'componentLabels', None)
    comp = {}

    if labels:
        # labels may be 'S11','S12' or 'E11','E23', etc.
        for i, lab in enumerate(labels):
            try:
                s = str(lab)
            except Exception:
                s = '%s' % lab
            if len(s) >= 2:
                key = s[-2:]  # '11','22','12','23',...
                if key in ('11', '22', '33', '12', '13', '23'):
                    try:
                        comp[key] = float(data[i])
                    except Exception:
                        comp[key] = 0.0
    else:
        # Fallback: Abaqus typical 3D order (11,22,33,12,13,23)
        if len(data) == 6:
            comp['11'] = float(data[0]); comp['22'] = float(data[1]); comp['33'] = float(data[2])
            comp['12'] = float(data[3]); comp['13'] = float(data[4]); comp['23'] = float(data[5])
        elif len(data) == 4:
            comp['11'] = float(data[0]); comp['22'] = float(data[1]); comp['33'] = float(data[2]); comp['12'] = float(data[3])
        elif len(data) == 3:
            comp['11'] = float(data[0]); comp['22'] = float(data[1]); comp['12'] = float(data[2])

    out = np.zeros(6, dtype=float)
    out[0] = comp.get('11', 0.0)
    out[1] = comp.get('22', 0.0)
    out[2] = comp.get('33', 0.0)
    out[3] = comp.get('23', 0.0)
    out[4] = comp.get('13', 0.0)
    out[5] = comp.get('12', 0.0)
    return out


# ----------------- Strict volume-weighted average (no mixed/unweighted averaging) -----------------
def _strict_volume_average_ip(frame, field_key):
    """
    Strict volume-weighted average for integration-point fields (field_key='S'/'E'/'LE').

    Policy:
      - Requires IVOL (integration point volume). No EVOL fallback.
      - Every integration point value must have a corresponding IVOL weight.
        If any weight is missing, raise an error to avoid mesh-dependent results.

    Returns: (avg6, Vsum, 'IVOL')
    """
    # Integration point values of the target field
    fo = frame.fieldOutputs[field_key].getSubset(position=INTEGRATION_POINT)
    vals = fo.values

    # Integration point volumes (required)
    if not _repo_has(frame.fieldOutputs, 'IVOL'):
        raise ValueError(
            "IVOL is not available in this frame. "
            "Please include IVOL in the Field Output Request."
        )
    iv = frame.fieldOutputs['IVOL'].getSubset(position=INTEGRATION_POINT)

    iv_map = {}
    for v in iv.values:
        iv_map[_ip_key(v)] = float(v.data)

    if not iv_map:
        raise ValueError(
            "IVOL field output exists but contains no integration-point values. "
            "Check your Field Output Request and element output settings."
        )

    s_sum = np.zeros(6, dtype=float)
    w_sum = 0.0
    missing_w = 0

    for v in vals:
        vec = _voigt6_from_field_value(fo, v)

        # Shear components (only meaningful for E/LE)
        if field_key in ('E', 'LE') and (not SHEAR_IS_ENGINEERING):
            vec[3] *= 2.0
            vec[4] *= 2.0
            vec[5] *= 2.0

        w = iv_map.get(_ip_key(v), None)
        if w is None:
            missing_w += 1
            continue

        s_sum += vec * w
        w_sum += w

    # Critical: forbid partially missing weights, otherwise results become mesh-dependent
    if missing_w:
        raise ValueError(
            "IVOL weights missing for %d/%d values in field %s. "
            "Mixed weighting is not allowed." % (missing_w, len(vals), field_key)
        )

    if w_sum <= 0.0:
        raise ValueError("Non-positive total IVOL weight (Vsum=%s) for field %s." % (w_sum, field_key))

    return (s_sum / w_sum, w_sum, 'IVOL')

def get_avg_stress(odb_path):
    """Return volume-averaged stress (Voigt): [sigma11, sigma22, sigma33, sigma23, sigma13, sigma12]."""
    from odbAccess import openOdb
    odb = None
    try:
        odb = openOdb(nstr(odb_path), readOnly=True)
        step_name, frame = _pick_last_frame(odb, odb_path)

        avgS, Vsum, mode = _strict_volume_average_ip(frame, 'S')

        if PRINT_POSTPROC_INFO:
            print("[post] %s step=%s  <S> ok  mode=%s  Vsum=%.6e"
                  % (os.path.basename(nstr(odb_path)), step_name, mode, Vsum))

        return avgS
    finally:
        try:
            if odb is not None:
                odb.close()
        except Exception:
            pass


def get_avg_strain(odb_path):
    """
    Return volume-averaged strain (Voigt): [e11, e22, e33, gamma23, gamma13, gamma12].
    Prefer E (small strain) by default; fall back to LE if missing (configurable).
    """
    from odbAccess import openOdb
    odb = None
    try:
        odb = openOdb(nstr(odb_path), readOnly=True)
        step_name, frame = _pick_last_frame(odb, odb_path)

        prefer = (STRAIN_FIELD or 'E').upper()
        if prefer == 'E':
            if _repo_has(frame.fieldOutputs, 'E'):
                key = 'E'
            elif STRAIN_ALLOW_FALLBACK and _repo_has(frame.fieldOutputs, 'LE'):
                key = 'LE'
                print("[Warn] %s: missing 'E', fallback to 'LE'"
                      % os.path.basename(nstr(odb_path)))
            else:
                raise ValueError("Frame missing 'E' (and fallback disabled): %s" % odb_path)
        elif prefer == 'LE':
            if _repo_has(frame.fieldOutputs, 'LE'):
                key = 'LE'
            elif STRAIN_ALLOW_FALLBACK and _repo_has(frame.fieldOutputs, 'E'):
                key = 'E'
                print("[Warn] %s: missing 'LE', fallback to 'E'"
                      % os.path.basename(nstr(odb_path)))
            else:
                raise ValueError("Frame missing 'LE' (and fallback disabled): %s" % odb_path)
        else:
            raise ValueError("Unknown STRAIN_FIELD=%s (use 'E' or 'LE')" % prefer)

        avgE, Vsum, mode = _strict_volume_average_ip(frame, key)

        if PRINT_POSTPROC_INFO:
            print("[post] %s step=%s  <%s> ok  mode=%s  Vsum=%.6e"
                  % (os.path.basename(nstr(odb_path)), step_name, key, mode, Vsum))

        return avgE
    finally:
        try:
            if odb is not None:
                odb.close()
        except Exception:
            pass


# ---------------- Macro strain (optional helper, kept for your interface) ----------------
def macro_strain_for_case_from_disp(case,
                                    Lx, Ly, Lz,
                                    dE11, dE22, dE33,
                                    dG12, dG13, dG23):
    """
    Convert opposite-face displacement differences for a load case into macro strain
    (Voigt: [e11, e22, e33, gamma23, gamma13, gamma12]).
    Note: compute_C_for_model currently uses <E> from the ODB; this helper is for cross-checking only.
    """
    eps = np.zeros(6, dtype=float)
    c = case.lower()

    if 'e11' in c:
        eps[0] = dE11 / float(Lx)
    elif 'e22' in c:
        eps[1] = dE22 / float(Ly)
    elif 'e33' in c:
        eps[2] = dE33 / float(Lz)
    elif 'g23' in c or 's23' in c:
        eps[3] = dG23 / float(Lz)
    elif 'g13' in c or 's13' in c:
        eps[4] = dG13 / float(Lz)
    elif 'g12' in c or 's12' in c:
        eps[5] = dG12 / float(Ly)
    else:
        raise ValueError('Unknown load case: %s' % case)

    return eps


# ---------------- Compute C for one model ----------------
def compute_C_for_model(model_name, work_dir):
    """Compute the effective stiffness matrix C (6x6) from 6 EasyPBC load-case ODBs.

    The ODB files can be either:
      - <model>_<case>.odb (preferred, created by this script), or
      - job-<case>.odb (fallback)

    Voigt order: [11, 22, 33, 23, 13, 12]
    """
    # 1) Collect ODB paths for the 6 load cases
    paths = []
    missing = []
    for case in CASES:
        p1 = os.path.join(work_dir, '%s_%s.odb' % (model_name, case))
        p2 = os.path.join(work_dir, 'job-%s.odb' % case)
        if os.path.exists(p1):
            paths.append((case, p1))
        elif os.path.exists(p2):
            paths.append((case, p2))
        else:
            missing.append(case)

    if missing:
        print('[Skip] %s missing ODB(s): %s' % (model_name, ', '.join(missing)))
        return None

    SIG = np.zeros((6, 6), dtype=float)  # columns = load cases
    EPS = np.zeros((6, 6), dtype=float)  # columns = load cases

    print('  Computing C for:', model_name)

    for case, odb in paths:
        s_avg = get_avg_stress(odb)  # [s11,s22,s33,s23,s13,s12]
        e_avg = get_avg_strain(odb)  # [e11,e22,e33,g23,g13,g12]

        col = CASES.index(case)  # fixed column index
        SIG[:, col] = s_avg
        EPS[:, col] = e_avg

        if PRINT_POSTPROC_INFO:
            print('    ', os.path.basename(odb))
            print('        avgS =', np.array2string(s_avg, precision=6))
            print('        avgE =', np.array2string(e_avg, precision=6))

    # 2) Compute C = SIG * inv(EPS)
    try:
        cond_eps = np.linalg.cond(EPS)
        print('  cond(EPS) = %.3e' % cond_eps)
    except Exception:
        pass

    try:
        EPS_inv = np.linalg.inv(EPS)
    except Exception as e:
        print('  [Warn] EPS is singular; using pseudo-inverse. Reason:', str(e))
        EPS_inv = np.linalg.pinv(EPS)

    C = np.dot(SIG, EPS_inv)

    # 3) Symmetrize (minor symmetry)
    Csym = 0.5 * (C + C.T)

    # 4) Save to disk (optional)
    try:
        np.savetxt(os.path.join(work_dir, 'C_%s.txt' % model_name), Csym, fmt='%.6e')
        np.save(os.path.join(work_dir, 'C_%s.npy' % model_name), Csym)
    except Exception as e:
        print('  [Warn] Failed to save C:', str(e))

    print('  C_eff (symmetrized) =')
    print(np.array2string(Csym, precision=6))
    return Csym

def export_C_table(work_dir, models):
    """Compute and export a summary CSV for all models."""
    rows = []

    for m in models:
        try:
            Csym = compute_C_for_model(m, work_dir)
            if Csym is None:
                continue
            rows.append([m] + [Csym[i, j] for i in range(6) for j in range(6)])
        except Exception as e:
            print('  [Error] Calculation failed for:', m, str(e))
            print(traceback.format_exc())
            continue

    if not rows:
        print('[WARN] No stiffness results were generated.')
        return None

    header = ['Model'] + ['C%d%d' % (i+1, j+1) for i in range(6) for j in range(6)]
    out_csv = os.path.join(work_dir, 'C_all_models.csv')

    # Abaqus/Py2: use 'wb'
    with open(nstr(out_csv), 'wb') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    print('[Summary CSV] ->', out_csv)
    return out_csv

def run_build_only(p):
    return build_models(p)


def run_easypbc_only(p, models=None):
    if not models:
        models = [m for m in mdb.models.keys() if m != 'Model-1']
    run_easypbc_for_models(p, models)
    return models


def run_compute_only(p, models=None):
    if not models:
        models = [m for m in mdb.models.keys() if m != 'Model-1']
    return export_C_table(p['WORK_DIR'], models)


def run_all(p):
    models = run_build_only(p)
    if not models:
        models = [m for m in mdb.models.keys() if m != 'Model-1']
    run_easypbc_only(p, models)
    run_compute_only(p, models)

def params_from_defaults(): return dict(DEFAULTS)

def _find_tk_icon_file():
    """Return an absolute icon path to use for the Tk window (or '' if none)."""
    # 1) User override
    if CUSTOM_TK_ICON:
        pth = os.path.join(THIS_DIR, nstr(CUSTOM_TK_ICON))
        if os.path.isfile(pth):
            return pth

    # 2) Auto-detect
    for fn in ICON_CANDIDATES:
        pth = os.path.join(THIS_DIR, nstr(fn))
        if os.path.isfile(pth):
            return pth
    return ''


def _apply_tk_icon(root):
    """Set the Tk window icon if an icon file is available next to this script."""
    if tk is None:
        return

    try:
        icon_path = _find_tk_icon_file()
        icon_path = nstr(icon_path)
        if (not icon_path) or (not os.path.isfile(icon_path)):
            return

        lower = icon_path.lower()

        # Best on Windows: .ico via iconbitmap
        if lower.endswith('.ico'):
            try:
                root.iconbitmap(default=icon_path)
                return
            except:
                pass

        # Try Pillow (supports png/jpg)
        try:
            from PIL import Image, ImageTk  # noqa: F401
            img = Image.open(icon_path)
            # Keep the icon reasonably small for Windows (taskbar/titlebar)
            try:
                img.thumbnail((64, 64), Image.ANTIALIAS)
            except:
                pass
            photo = ImageTk.PhotoImage(img)
            try:
                # Prefer low-level call for older Tk builds
                root.tk.call('wm', 'iconphoto', root._w, photo)
            except:
                try:
                    if hasattr(root, 'iconphoto'):
                        root.iconphoto(True, photo)
                except:
                    pass
            root._beaver_icon_ref = photo  # keep a reference
            return
        except:
            pass

        # Fallback: Tk PhotoImage (png support depends on the Tk build)
        try:
            photo = tk.PhotoImage(file=icon_path)

            # Downscale very large images using subsample (if needed)
            try:
                w = int(photo.width())
                h = int(photo.height())
                max_side = max(w, h)
                if max_side > 64:
                    factor = int(math.ceil(float(max_side) / 64.0))
                    if factor > 1:
                        photo = photo.subsample(factor, factor)
            except:
                pass

            try:
                root.tk.call('wm', 'iconphoto', root._w, photo)
            except:
                try:
                    if hasattr(root, 'iconphoto'):
                        root.iconphoto(True, photo)
                except:
                    pass

            root._beaver_icon_ref = photo
        except:
            # If you only have a PNG and your Tk does not support it,
            # convert it to ICO or GIF and place it next to Beaver.py.
            pass
    except:
        pass




def launch_gui():
    """Tkinter GUI entry point."""
    if tk is None:
        print('[WARN] Tkinter is not available; running in batch mode.')
        p = params_from_defaults()
        init_logging(p['WORK_DIR'])
        try:
            run_all(p)
        except Exception as e:
            print('[%s] Batch run failed:' % APP_NAME, e)
            print(traceback.format_exc())
        finally:
            end_logging()
            if p.get('AUTO_EXIT', False):
                safe_exit()
        return

    p = params_from_defaults()

    root = tk.Tk()
    root.title('%s - RVE Automation' % APP_NAME)
    _apply_tk_icon(root)

    # ---------- Layout helpers ----------
    def add_row(parent, r, label, key, width=40, browse=None):
        tk.Label(parent, text=label, anchor='e').grid(row=r, column=0, sticky='e', padx=4, pady=2)
        var = tk.StringVar(value=str(p.get(key, '')))
        ent = tk.Entry(parent, textvariable=var, width=width)
        ent.grid(row=r, column=1, sticky='w', padx=4, pady=2)

        def pick_dir():
            initdir = var.get() if os.path.isdir(var.get()) else os.getcwd()
            d = tkFileDialog.askdirectory(initialdir=initdir)
            if d:
                var.set(d)

        def pick_file():
            initdir = p.get('WORK_DIR', os.getcwd())
            f = tkFileDialog.asksaveasfilename(initialdir=initdir,
                                               initialfile=p.get(key, ''),
                                               defaultextension='.cae')
            if f:
                var.set(f)

        if browse == 'dir':
            tk.Button(parent, text='Browse', command=pick_dir).grid(row=r, column=2, padx=2)
        elif browse == 'file':
            tk.Button(parent, text='Save As', command=pick_file).grid(row=r, column=2, padx=2)
        return var

    # ---------- Frames ----------
    frm_io = tk.LabelFrame(root, text='I/O', padx=6, pady=6)
    frm_io.grid(row=0, column=0, columnspan=3, sticky='ew', padx=8, pady=6)

    v_work = add_row(frm_io, 0, 'Work directory (ODB/CSV)', 'WORK_DIR', browse='dir')
    v_cae  = add_row(frm_io, 1, 'CAE file name', 'SAVE_CAE_NAME', browse='file')
    v_stl  = add_row(frm_io, 2, 'STL input directory', 'STL_DIR', browse='dir')

    frm_geo = tk.LabelFrame(root, text='Geometry / Mesh', padx=6, pady=6)
    frm_geo.grid(row=1, column=0, columnspan=3, sticky='ew', padx=8, pady=6)

    v_lx = add_row(frm_geo, 0, 'Cube Lx', 'Lx')
    v_ly = add_row(frm_geo, 1, 'Cube Ly', 'Ly')
    v_lz = add_row(frm_geo, 2, 'Cube Lz', 'Lz')
    v_ms = add_row(frm_geo, 3, 'Mesh size', 'MESH_SIZE')
    v_sh = add_row(frm_geo, 4, 'Max shrink (0.05 = 5%)', 'MAX_SHRINK')

    frm_mat = tk.LabelFrame(root, text='Material (Elastic)', padx=6, pady=6)
    frm_mat.grid(row=2, column=0, columnspan=3, sticky='ew', padx=8, pady=6)

    v_me  = add_row(frm_mat, 0, 'Matrix E',  'MAT_MATRIX_E')
    v_mnu = add_row(frm_mat, 1, 'Matrix nu', 'MAT_MATRIX_NU')
    v_fe  = add_row(frm_mat, 2, 'Fibre E',   'MAT_FIBRE_E')
    v_fnu = add_row(frm_mat, 3, 'Fibre nu',  'MAT_FIBRE_NU')

    frm_ep = tk.LabelFrame(root, text='EasyPBC', padx=6, pady=6)
    frm_ep.grid(row=3, column=0, columnspan=3, sticky='ew', padx=8, pady=6)

    v_cpu = add_row(frm_ep, 0, 'CPU count', 'EASYPBC_CPU')
    v_msens = add_row(frm_ep, 1, 'Mesh sensitivity', 'EASYPBC_MESHSENS')

    # ---------- Param sync ----------
    def sync_params():
        p['WORK_DIR'] = nstr(v_work.get())
        p['SAVE_CAE_NAME'] = nstr(os.path.basename(v_cae.get()))
        p['STL_DIR'] = nstr(v_stl.get())

        p['Lx'] = float(v_lx.get())
        p['Ly'] = float(v_ly.get())
        p['Lz'] = float(v_lz.get())
        p['MESH_SIZE'] = float(v_ms.get())
        p['MAX_SHRINK'] = float(v_sh.get())

        p['MAT_MATRIX_E'] = float(v_me.get())
        p['MAT_MATRIX_NU'] = float(v_mnu.get())
        p['MAT_FIBRE_E'] = float(v_fe.get())
        p['MAT_FIBRE_NU'] = float(v_fnu.get())

        p['EASYPBC_CPU'] = int(float(v_cpu.get()))
        p['EASYPBC_MESHSENS'] = float(v_msens.get())

        # Verify local plug-in modules early (more user-friendly)
        ensure_runtime_dependencies(strict=True)
        return p

    # ---------- Button callbacks ----------
    def _run_with_log(title, fn):
        sync_params()
        log_path = init_logging(p['WORK_DIR'])
        try:
            print('[%s] %s  Log: %s' % (APP_NAME, title, log_path))
            fn(p)
            print('[%s] %s DONE' % (APP_NAME, title))
        except Exception as e:
            print('[%s] %s FAILED: %s' % (APP_NAME, title, str(e)))
            print(traceback.format_exc())
        finally:
            end_logging()

    def on_build():
        _run_with_log('BUILD ONLY', run_build_only)

    def on_epbc():
        _run_with_log('EASYPBC ONLY', run_easypbc_only)

    def on_compute():
        _run_with_log('STIFFNESS ONLY', run_compute_only)

    def on_all():
        sync_params()
        log_path = init_logging(p['WORK_DIR'])
        try:
            print('[%s] RUN ALL  Log: %s' % (APP_NAME, log_path))
            run_all(p)
            print('[%s] RUN ALL DONE' % APP_NAME)
        except Exception as e:
            print('[%s] RUN ALL FAILED: %s' % (APP_NAME, str(e)))
            print(traceback.format_exc())
        finally:
            end_logging()
            try:
                root.destroy()
            except:
                pass
            if p.get('AUTO_EXIT', False):
                safe_exit()

    # ---------- Buttons ----------
    frm_btn = tk.Frame(root, padx=8, pady=8)
    frm_btn.grid(row=4, column=0, columnspan=3, sticky='ew')

    tk.Button(frm_btn, text='Build Only', width=14, command=on_build).grid(row=0, column=0, padx=4, pady=4)
    tk.Button(frm_btn, text='EasyPBC Only', width=14, command=on_epbc).grid(row=0, column=1, padx=4, pady=4)
    tk.Button(frm_btn, text='Stiffness Only', width=14, command=on_compute).grid(row=0, column=2, padx=4, pady=4)
    tk.Button(frm_btn, text='Run All', width=14, command=on_all).grid(row=0, column=3, padx=4, pady=4)

    def on_close():
        try:
            end_logging()
        except:
            pass
        try:
            root.destroy()
        except:
            pass
        if p.get('AUTO_EXIT', False):
            safe_exit()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


# Entry point

# Integrated EasyPBC code (patched for Beaver integration)
# - Adds IVOL to field output requests (for strict volume weighting)
# - Adds reusePBC flag: if PBC sets already exist, skip set recreation
###############################################################################

##      EasyPBC Ver. 1.4   (08/10/2018) updated on (27/08/2019) to calculte CTE and fix work directory change error.
##      EasyPBC is an ABAQUS CAE plugin developed to estimate the homogenised effective elastic properties of user created periodic(RVE)
##      Copyright (C) 2018  Sadik Lafta Omairey
##
##      This program is distributed in the hope that it will be useful,
##      but WITHOUT ANY WARRANTY; without even the implied warranty of
##      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##      GNU General Public License for more details.
##
##      You should have received a copy of the GNU General Public License
##      along with this program.  If not, see <https://www.gnu.org/licenses/>.
##      Kindly do not re-distribute as it is subjected to updates.
##
##      Citation: Omairey S, Dunning P, Sriramula S (2018) Development of an ABAQUS plugin tool for periodic RVE homogenisation.
##      Engineering with Computers. https://doi.org/10.1007/s00366-018-0616-4
##      Email sadik.omairey@gmail.com to obtain the latest version of the software.




## Importing ABAQUS Data and Python modules ##

from abaqus import *
from abaqusConstants import *
import __main__
import math
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
import time
import os
import sys
import ctypes
import multiprocessing

## Plugin main GUI function ##

def feasypbc(part,inst,meshsens,E11,E22,E33,G12,G13,G23,CTE,CPU,onlyPBC, intemp, fntemp, reusePBC=True, requestIVOL=True):
        import os
        path = os.getcwd()
        for T in (range(1)):
                start = time.time()
                modelName = part
                instanceName = inst
                upperName= inst.upper()
                
                fail = []
                keycheck2 =[inst]
                
                if part not in (mdb.models.keys()):
                        Er2=0
                        messageBox2 = ctypes.windll.user32.MessageBoxA
                        returnValue = messageBox2(Er2,'Model name is incorrect, please input the correct Model name.','EasyPBC Start-up error 02',0x30 | 0x0)
                        print('Start-up error 02. Refer EasyPBC user guide')
                        continue
                
                a = mdb.models[modelName].rootAssembly
                # Beaver integration: detect existing EasyPBC setup to avoid duplicate set creation
                pbc_exists = False
                if reusePBC:
                        try:
                                sets_repo = a.sets
                                # Minimal marker sets created by EasyPBC (2D/3D)
                                marker_names = ('fronts','backs','tops','bots','c1','c2','c5','c6','RP1','RP2','RP3')
                                ok = True
                                for _n in marker_names:
                                        if not sets_repo.has_key(_n):
                                                ok = False
                                                break
                                # If the 3D face sets exist, also require the remaining corners and RPs
                                if ok and sets_repo.has_key('lefts') and sets_repo.has_key('rights'):
                                        for _n in ('c3','c4','c7','c8','RP4','RP5','RP6'):
                                                if not sets_repo.has_key(_n):
                                                        ok = False
                                                        break
                                if ok:
                                        # Require at least one single-node set (e.g. 'fronts123')
                                        has_single = False
                                        try:
                                                for _k in sets_repo.keys():
                                                        if _k.startswith('fronts') and _k != 'fronts':
                                                                has_single = True
                                                                break
                                        except:
                                                has_single = True
                                        if has_single:
                                                pbc_exists = True
                        except:
                                pbc_exists = False

                errorcheck1 = mdb.models[modelName].rootAssembly.instances.keys()
                if errorcheck1 == fail:
                        Er1=0
                        messageBox1 = ctypes.windll.user32.MessageBoxA
                        returnValue = messageBox1(Er1,'Model part is not created!\nPlease create part and try again','EasyPBC Start-up error 01',0x30 | 0x0)
                        print('Start-up error 01. Refer EasyPBC user guide')
                        continue
                
                if (mdb.models[modelName].rootAssembly.instances.keys()) != keycheck2:                       
                        Er3=0
                        messageBox3 = ctypes.windll.user32.MessageBoxA
                        returnValue = messageBox3(Er3,'Instance name is incorrect, please input the correct instance name.','EasyPBC Start-up error 03',0x30 | 0x0)
                        print('Start-up error 03. Refer EasyPBC user guide')
                        continue

                if CPU <= 0:
                        Er5=0
                        messageBox5 = ctypes.windll.user32.MessageBoxA
                        returnValue = messageBox5(Er5,'Specified number of CPUs is <= zero, please set it to a value larger than zero.','EasyPBC Start-up error 05',0x30 | 0x0)
                        print('Start-up error 05. Refer EasyPBC user guide')
                        continue

                
                CPUs = int(round(CPU))
                if CPUs > multiprocessing.cpu_count():
                        CPUs = multiprocessing.cpu_count()
                        print ('Warning: Specified number of CPUs is greater than the available. The maximum available number of CPUs is used (%s CPU(s)).' % CPUs)


                Nodeset = mdb.models[modelName].rootAssembly.instances[instanceName].nodes

                ## Start of sets creation ##                
                j = 0
                x=[]
                y=[]
                z=[]
                c1=[]
                c2=[]
                c3=[]
                c4=[]
                c5=[]
                c6=[]
                c7=[]
                c8=[]
                Max=[]
                ftedgexyz={}
                btedgexyz={}
                fbedgexyz={}
                bbedgexyz={}
                fledgexyz={}
                bledgexyz={}
                fredgexyz={}
                bredgexyz={}
                ltedgexyz={}
                rtedgexyz={}
                lbedgexyz={}
                rbedgexyz={}
                frontsxyz={}
                backsxyz={}
                topsxyz={}
                botsxyz={}
                leftsxyz={}
                rightsxyz={}
                frontbcxyz={}
                backbcxyz={}
                topbcxyz={}
                botbcxyz={}
                leftbcxyz={}
                rightbcxyz={}
                ftedge=[]
                btedge=[]
                fbedge=[]
                bbedge=[]
                fledge=[]
                fredge=[]
                bledge=[]
                bredge=[]
                ltedge=[]
                lbedge=[]
                rtedge=[]
                rbedge=[]
                fronts=[]
                backs=[]
                lefts=[]
                rights=[]
                tops=[]
                bots=[]
                backs=[]
                frontbc=[]
                backbc=[]
                leftbc=[]
                rightbc=[]
                topbc=[]
                botbc=[]
                backbc=[]
                errorset=[]
                coc1={}
                coc2={}
                coc3={}
                coc4={}
                coc5={}
                coc6={}
                coc7={}
                coc8={}

                error=False


                print ('----------------------------------')
                print ('-------- Start of EasyPBC --------')
                print ('----------------------------------')



                ## Identifying RVE size ##    
                for i in Nodeset:
                    x.insert(j,i.coordinates[0])
                    y.insert(j,i.coordinates[1])
                    z.insert(j,i.coordinates[2])
                    j=j+1



                errorcheck4 = x                   
                if not errorcheck4:
                        Er4=0
                        messageBox4 = ctypes.windll.user32.MessageBoxA
                        returnValue = messageBox4(Er4,'Instance not detected! Make sure:\n1- Instance is created;\n2- Double click on instance to refresh it before running EasyPBC;\n3- Part/instnace is meshed.','EasyPBC Start-up error 04',0x30 | 0x0)
                        print('Start-up error 04. Refer EasyPBC user guide')
                        continue


                Max = max(x)
                May = max(y)
                Maz = max(z)
                Mnx = min(x)
                Mny = min(y)
                Mnz = min(z)


                
                if (Maz - Mnz)<=meshsens:  ## 2D Model Check

                ############################################ 2D Section #####################################

                        L=abs(Max-Mnx)
                        H=abs(May-Mny)
                        
                        Dispx = L*0.2
                        Dispy = H*0.2

                        #### This part is commented out to reduce complexity for users whom merge parts into a single instance. ##############
                        
##                        partName = mdb.models[modelName].rootAssembly.instances[inst].partName                ## Extracts part name used in the instance
##                        AssSec = mdb.models[modelName].parts[partName].sectionAssignments[0].sectionName
##                        if AssSec == None:
##                                print 'Warning: No shell sections detected, please create a section and try again!!'
##                                print '         Refer to error 09 troubleshooting in easyPBC user guide.'
##                                error=True
##                                continue

                        SecName = mdb.models['Model-1'].sections.keys()[0]                                      ## Finding the name of first section            
                        Thikness = mdb.models[modelName].sections[SecName].thickness
                        skipAtt = False
                        
                        if Thikness == None or Thikness == 0:
                                Thikness = 1
                                print('Attention: EasyPBC did not detected a shell thickness. Thus, it assumes thickness is equal to 1.0 unit length')
                                print('           If another thickness value is desired, divide the elastic property(ies) by the actual thickness value.')
                                skipAtt = True
                        if skipAtt == False:
                                print('Attention: EasyPBC detected a shell thickness of %s unit length from the first section in ABAQUS property module.' % Thikness)
                                print('           If this value is incorrect (this section is not used), multiply the elastic property(ies) %s and divide it by the actual value.' % Thikness)

                        
                        ## Creating Ref. Points (Same R.Ps as in the 3D case) ##
                        for i in a.features.keys():
                            if i.startswith('RP'):
                                del a.features['%s' % (i)]
                        a.ReferencePoint(point=(Max+0.8*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP6: G23
                        a.ReferencePoint(point=(Max+0.6*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP5: G13
                        a.ReferencePoint(point=(Max+0.4*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP4: G12
                        a.ReferencePoint(point=(Max+0.2*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP3: Rigid body movement X-axis
                        a.ReferencePoint(point=(Max-0.5*(Max-Mnx), May-0.5*(May-Mny), Maz+0.2*abs(Maz-Mnz)))  ## RP2: Rigid body movement Z-axis
                        a.ReferencePoint(point=(Max-0.5*(Max-Mnx), May+0.2*abs(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP1: Rigid body movement Y-axis

                        r1 = a.referencePoints

                        ## Naming Ref. Points ##
                        d=1
                        for i in r1.keys():
                            refPoints1=(r1[i], )
                            if a.sets.has_key('RP%s' % (d)):
                                    del a.sets['RP%s' % (d)]
                            a.Set(referencePoints=refPoints1, name='RP%s' % (d))
                            d=d+1
                          
                        ## Identifying boundary nodes ##
                        for i in Nodeset:
                            if (Mnx+meshsens) < i.coordinates[0] < (Max-meshsens) and (Mny+meshsens) < i.coordinates[1] < (May-meshsens):
                                continue

                            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)<=meshsens:
                                c1.insert(0,i.label)
                                coc1[i.label]=[i.coordinates[0], i.coordinates[1]]
                            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)<=meshsens:
                                c2.insert(0,i.label)
                                coc2[i.label]=[i.coordinates[0], i.coordinates[1]]
                            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens:
                                c5.insert(0,i.label)
                                coc5[i.label]=[i.coordinates[0], i.coordinates[1]]
                            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens:
                                c6.insert(0,i.label)
                                coc6[i.label]=[i.coordinates[0], i.coordinates[1]]
                            if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                                frontsxyz[i.label]=[i.coordinates[0], i.coordinates[1]]
                            if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                                backsxyz[i.label]=[i.coordinates[0], i.coordinates[1]] 
                            if abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                                topsxyz[i.label]=[i.coordinates[0], i.coordinates[1]]
                            if abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                                botsxyz[i.label]=[i.coordinates[0], i.coordinates[1]]

                        ## Checking number of nodes of opposite/associated sets ##
                        if len(frontsxyz) != len(backsxyz):
                         print('Warning: Number of Nodes in Front surface (fronts) not equal to number of nodes in Back surface (backs). These sets will not be created!!')
                         print('         Refer to error 06 troubleshooting in easyPBC user guide.')
                         frontsxyz={}
                         error=True
                        if len(topsxyz) != len(botsxyz):
                         print('Warning: Number of Nodes in Top surface (tops) not equal to number of nodes in Bottom surface (bots). These sets will not be created!!')
                         print('         Refer to error 06 in easyPBC user guide.')
                         topsxyz={}
                         error=True
       
                        ## Sorting and appending sets ##
                        for i in frontsxyz.keys():
                                for k in backsxyz.keys():
                                        if abs(frontsxyz[i][1] - backsxyz[k][1])<=meshsens:
                                                fronts.append(i)
                                                backs.append(k)

                        if len(frontsxyz)!= len(fronts) or len(backsxyz)!= len(backs):
                            print('Warning: Node(s) in Front and/or Back surface (fronts and/or backs) was not imported. effected sets will not be created!!')
                            print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                            for i, k in zip(frontsxyz.keys(),backsxyz.keys()):
                                    if i not in fronts:
                                            errorset.append(i)
                                    if k not in backs:
                                            errorset.append(k)
                            fronts=[]
                            backs=[]
                            error=True                    
                        if len(fronts)!=len(set(fronts)) or len(backs)!=len(set(backs)):
                            print('Warning: Node(s) in either Front or Back surface (fronts or backs) being linked with more than one opposite node. effected sets will not be created!!')
                            print('         Refer to error 08 in easyPBC user guide.')
                            fronts=[]
                            backs=[]
                            error=True

                        for i in topsxyz.keys():
                            for k in botsxyz.keys():
                                if abs(topsxyz[i][0] - botsxyz[k][0]) <=meshsens:
                                    tops.append(i)
                                    bots.append(k)
                        if len(topsxyz)!= len(tops) or len(botsxyz)!= len(bots):
                            print('Warning: Node(s) in Top and/or Bottom surface (tops and/or bots) was not imported. effected sets will not be created!!')
                            print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                            for i, k in zip(topsxyz.keys(),botsxyz.keys()):
                                    if i not in tops:
                                            errorset.append(i)
                                    if k not in bots:
                                            errorset.append(k)
                            tops=[]
                            bots=[]
                            error=True
                        if len(tops)!=len(set(tops)) or len(bots)!=len(set(bots)):
                            print('Warning: Node(s) in either Top or Bottom surface (tops or bots) being linked with more than one opposite node. effected sets will not be created!!')
                            print('         Refer to error 08 in easyPBC user guide.')
                            tops=[]
                            bots=[]
                            error=True



                        if (reusePBC and pbc_exists):
                                print ('------ EasyPBC sets already exist: skip sets creation ------')
                        else:
                                ## Creating ABAQUS sets ##
                                a.SetFromNodeLabels(name='c1', nodeLabels=((instanceName,c1),))
                                a.SetFromNodeLabels(name='c2', nodeLabels=((instanceName,c2),))
                                a.SetFromNodeLabels(name='c5', nodeLabels=((instanceName,c5),))
                                a.SetFromNodeLabels(name='c6', nodeLabels=((instanceName,c6),))
                                a.SetFromNodeLabels(name='fronts', nodeLabels=((instanceName,fronts),))
                                a.SetFromNodeLabels(name='backs', nodeLabels=((instanceName,backs),))
                                a.SetFromNodeLabels(name='tops', nodeLabels=((instanceName,tops),))
                                a.SetFromNodeLabels(name='bots', nodeLabels=((instanceName,bots),))
                                print ('------ End of Sets Creation ------')

                        ## Extracting model mass ##
                        prop=mdb.models[modelName].rootAssembly.getMassProperties()
                        mass=prop['mass']

                        a = mdb.models[modelName].rootAssembly
                        Nodeset = mdb.models[modelName].rootAssembly.instances[instanceName].nodes
                        if not mdb.models[modelName].steps.has_key('Step-1'):
                                mdb.models[modelName].StaticStep(name='Step-1', previous='Initial')
                        # Ensure IVOL is available for strict volume-weighted post-processing
                        if requestIVOL:
                                try:
                                        _fo_name = 'F-IVOL'
                                        _reqs = mdb.models[modelName].fieldOutputRequests
                                        if _fo_name in _reqs.keys():
                                                try:
                                                        _reqs[_fo_name].setValues(variables=('IVOL',))
                                                except:
                                                        pass
                                        else:
                                                mdb.models[modelName].FieldOutputRequest(name=_fo_name, createStepName='Step-1', variables=('IVOL',))
                                except Exception as _e:
                                        # Fallback: try to append IVOL to 'F-Output-1' (handles PRESELECT safely)
                                        try:
                                                if 'F-Output-1' in mdb.models[modelName].fieldOutputRequests.keys():
                                                        _fo = mdb.models[modelName].fieldOutputRequests['F-Output-1']
                                                        _v0 = getattr(_fo, 'variables', None)
                                                        if _v0 == PRESELECT:
                                                                _fo.setValues(variables=('S','E','LE','U','RF','IVOL'))
                                                        else:
                                                                try:
                                                                        _vlist = list(_v0) if hasattr(_v0, '__iter__') else []
                                                                except:
                                                                        _vlist = []
                                                                if 'IVOL' not in _vlist:
                                                                        _vlist.append('IVOL')
                                                                try:
                                                                        _fo.setValues(variables=tuple(_vlist))
                                                                except:
                                                                        pass
                                                else:
                                                        mdb.models[modelName].FieldOutputRequest(name='F-Output-1', createStepName='Step-1', variables=('S','E','LE','U','RF','IVOL'))
                                        except:
                                                pass

                        ## Creating single-node ABAQUS sets ##                
                        if error==False:
                                
                                if (reusePBC and pbc_exists):
                                        # Single-node sets already exist from a previous run
                                        pass
                                else:
                                        if E11==True or E22==True or E33==True or G12==True or G13==True or G23==True:
                                                for i,k in zip(tops,bots):
                                                    a.SetFromNodeLabels(name='tops%s' % (i), nodeLabels=((instanceName,[i]),))
                                                    a.SetFromNodeLabels(name='bots%s' % (k), nodeLabels=((instanceName,[k]),))
        
                                                for i,k in zip(fronts,backs):
                                                    a.SetFromNodeLabels(name='fronts%s' % (i), nodeLabels=((instanceName,[i]),))
                                                    a.SetFromNodeLabels(name='backs%s' % (k), nodeLabels=((instanceName,[k]),))
        
        
                                ## Creating constraints for elastic moduli ##
                                if E11==True or E22==True:
                                        for i in mdb.models[modelName].constraints.keys():
                                                del mdb.models[modelName].constraints[i]
                                                
                                        for i,k in zip(tops,bots):
                                            mdb.models[modelName].Equation(name='E-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1)))
                                        for i,k in zip(tops,bots):
                                            mdb.models[modelName].Equation(name='E-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2),(-1.0, 'RP5', 2)))


                                        for i,k in zip(fronts,backs):
                                            mdb.models[modelName].Equation(name='E-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1),(-1.0, 'RP4', 1)))
                                        for i,k in zip(fronts,backs):
                                            mdb.models[modelName].Equation(name='E-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2)))


                                        mdb.models[modelName].Equation(name='E-1-c62', terms=((1.0, 'c6', 1), (-1.0, 'c2', 1)))
                                        mdb.models[modelName].Equation(name='E-1-c21', terms=((1.0, 'c2', 1), (-1.0, 'c1', 1),(1.0, 'RP4', 1)))
                                        mdb.models[modelName].Equation(name='E-1-c15', terms=((1.0, 'c1', 1), (-1.0, 'c5', 1)))
                                       
                                        mdb.models[modelName].Equation(name='E-2-c62', terms=((1.0, 'c6', 2), (-1.0, 'c2', 2),(1.0, 'RP5', 2)))
                                        mdb.models[modelName].Equation(name='E-2-c21', terms=((1.0, 'c2', 2), (-1.0, 'c1', 2)))
                                        mdb.models[modelName].Equation(name='E-2-c15', terms=((1.0, 'c1', 2), (-1.0, 'c5', 2),(-1.0, 'RP5', 2)))


                                ## Elastic modulus E11 ##
                                if E11==True and onlyPBC == False:
                                        for i in mdb.models[modelName].loads.keys():
                                                del mdb.models[modelName].loads[i]
                                        for i in mdb.models[modelName].boundaryConditions.keys():
                                                del mdb.models[modelName].boundaryConditions[i]


                                        region = a.sets['RP4']
                                        mdb.models[modelName].DisplacementBC(name='E11-1', createStepName='Step-1', 
                                            region=region, u1=Dispx, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                            amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                            localCsys=None)

                                        regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                        if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                                del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                        mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                            createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                            sectionPoints=DEFAULT, rebar=EXCLUDE)

                                        import os, glob

                                        if mdb.jobs.has_key('job-E11'):
                                                del mdb.jobs['job-E11']
                                        mdb.Job(name='job-E11', model= modelName, description='', type=ANALYSIS, 
                                            atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                            memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                            explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                            modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                            scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                        mdb.jobs['job-E11'].submit(consistencyChecking=OFF)
                                        mdb.jobs['job-E11'].waitForCompletion()
                                        o3 = session.openOdb(name='%s' % (path+'\job-E11.odb'))
                                     
                                        odb = session.odbs['%s' % (path+'\job-E11.odb')]

                                        session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                        odbName=session.viewports[session.currentViewportName].odbDisplay.name



                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]

                                        session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                        session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                            NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP4', ))

                                        forceE11 = 0
                                        for i in session.xyDataObjects.keys():
                                            forceE11=forceE11+(session.xyDataObjects[i][0][1])

                                        stressE11 = abs(forceE11/(H*Thikness))


                                        E11 = stressE11/(Dispx/L)                               


                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]
                                        
                                        session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                        session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                            NODAL, ((COMPONENT, 'U1'), )), ), nodeSets=('C1','C2', ))
                                        
                                        C1U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][0]
                                        
                                        C2U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c2[0])][0][1] + coc2[(c2[0])][0]
                                        Dis = abs(C1U1new - C2U1new)

                                        E11U1= abs(L - Dis)


                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]
                                            

                                        session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                        session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                            NODAL, ((COMPONENT, 'U2'), )), ), nodeSets=('C1','C5', ))
                                        
                                        C1U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][1]
                                        
                                        C5U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c5[0])][0][1] + coc5[(c5[0])][1]
                                        Dis = abs(C1U2new - C5U2new)

                                        E11U2= abs(H - Dis)

                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]



                                        V12=(E11U2/H)/(E11U1/L)


                                ## Elastic modulus E22 ##                                
                                if E11==False or onlyPBC == True:
                                        E11='N/A'
                                        V12='N/A'

                                if E22==True and onlyPBC == False:

                                        for i in mdb.models[modelName].loads.keys():
                                                del mdb.models[modelName].loads[i]
                                        for i in mdb.models[modelName].boundaryConditions.keys():
                                                del mdb.models[modelName].boundaryConditions[i]

                                        region = a.sets['RP5']
                                        mdb.models[modelName].DisplacementBC(name='E22-1', createStepName='Step-1', 
                                            region=region, u1=UNSET, u2=Dispy, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                            amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                            localCsys=None)


                                        regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                        if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                                del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                        mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                            createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                            sectionPoints=DEFAULT, rebar=EXCLUDE)

                                        import os, glob

                                        if mdb.jobs.has_key('job-E22'):
                                                del mdb.jobs['job-E22']
                                        mdb.Job(name='job-E22', model= modelName, description='', type=ANALYSIS, 
                                            atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                            memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                            explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                            modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                            scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                        mdb.jobs['job-E22'].submit(consistencyChecking=OFF)
                                        mdb.jobs['job-E22'].waitForCompletion()
                                        o3 = session.openOdb(name='%s' % (path+'\job-E22.odb'))
                                     
                                        odb = session.odbs['%s' % (path+'\job-E22.odb')]

                                        session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                        odbName=session.viewports[session.currentViewportName].odbDisplay.name


                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]

                                        session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                        session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                            NODAL, ((COMPONENT, 'RF2'), )), ), nodeSets=('RP5', ))

                                        forceE22 = 0
                                        for i in session.xyDataObjects.keys():
                                            forceE22=forceE22+(session.xyDataObjects[i][0][1])

                                        stressE22 = abs(forceE22/(L*Thikness))


                                        E22 = stressE22/(Dispy/H)                                




                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]


                                        
                                        session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                        session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                            NODAL, ((COMPONENT, 'U1'), )), ), nodeSets=('C1','C2', ))
                                        
                                        C1U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][0]
                                        
                                        C2U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c2[0])][0][1] + coc2[(c2[0])][0]
                                        Dis = abs(C1U1new - C2U1new)

                                        E22U1= abs(L - Dis)


                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]
                                            

                                        session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                        session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                            NODAL, ((COMPONENT, 'U2'), )), ), nodeSets=('C1','C5', ))
                                        
                                        C1U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][1]
                                        
                                        C5U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c5[0])][0][1] + coc5[(c5[0])][1]
                                        Dis = abs(C1U2new - C5U2new)

                                        E22U2= abs(H - Dis)

                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]




                                        V21=(E22U1/L)/(E22U2/H)


                                ## Elastic modulus E33 ##
                                if E22==False or onlyPBC == True:
                                        E22='N/A'
                                        V21='N/A'



                                ## Creating constraints for shear moduli ##
                                if G12==True:
                                        if onlyPBC == False:
                                                for i in mdb.models[modelName].constraints.keys():
                                                        del mdb.models[modelName].constraints[i]

                                        for i,k in zip(tops,bots):
                                            mdb.models[modelName].Equation(name='G-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1),(-1.0, 'RP4', 1)))
                                        for i,k in zip(tops,bots):
                                            mdb.models[modelName].Equation(name='G-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2),(-1.0, 'RP1', 2)))
                                       

                                        for i,k in zip(fronts,backs):
                                            mdb.models[modelName].Equation(name='G-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1),(-1.0, 'RP3', 1)))
                                        for i,k in zip(fronts,backs):
                                            mdb.models[modelName].Equation(name='G-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2),(-1.0, 'RP4', 2)))


                                        mdb.models[modelName].Equation(name='G-1-c62', terms=((1.0, 'c6', 1), (-1.0, 'c2', 1),(1.0, 'RP4', 1)))
                                        mdb.models[modelName].Equation(name='G-1-c21', terms=((1.0, 'c2', 1), (-1.0, 'c1', 1),(1.0, 'RP3', 1)))
                                        mdb.models[modelName].Equation(name='G-1-c15', terms=((1.0, 'c1', 1), (-1.0, 'c5', 1),(-1.0, 'RP4', 1)))

                                            
                                        mdb.models[modelName].Equation(name='G-2-c62', terms=((1.0, 'c6', 2), (-1.0, 'c2', 2),(1.0, 'RP1', 2)))
                                        mdb.models[modelName].Equation(name='G-2-c21', terms=((1.0, 'c2', 2), (-1.0, 'c1', 2),(1.0, 'RP4', 2)))
                                        mdb.models[modelName].Equation(name='G-2-c15', terms=((1.0, 'c1', 2), (-1.0, 'c5', 2),(-1.0, 'RP1', 2)))


                                ## Shear modulus G12 ##
                                if G12==True and onlyPBC == False:
                                        for i in mdb.models[modelName].loads.keys():
                                                del mdb.models[modelName].loads[i]
                                        for i in mdb.models[modelName].boundaryConditions.keys():
                                                del mdb.models[modelName].boundaryConditions[i]

                                        region = a.sets['RP4']
                                        mdb.models[modelName].DisplacementBC(name='G12-1', createStepName='Step-1', 
                                            region=region, u1=Dispx, u2=Dispy, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                            amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                            localCsys=None)


                                        regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                        if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                                del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                        mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                            createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                            sectionPoints=DEFAULT, rebar=EXCLUDE)

                                        import os, glob

                                        if mdb.jobs.has_key('job-G12'):
                                                del mdb.jobs['job-G12']
                                        mdb.Job(name='job-G12', model= modelName, description='', type=ANALYSIS, 
                                            atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                            memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                            explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                            modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                            scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                        mdb.jobs['job-G12'].submit(consistencyChecking=OFF)

                                        mdb.jobs['job-G12'].waitForCompletion()


                                        o3 = session.openOdb(name='%s' % (path+'\job-G12.odb'))


                                        odb = session.odbs['%s' % (path+'\job-G12.odb')]

                                        session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                        odbName=session.viewports[session.currentViewportName].odbDisplay.name


                                        for i in session.xyDataObjects.keys():
                                            del session.xyDataObjects['%s' % (i)]

                                        session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                        session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                            NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP4', ))

                                        forceG12 = 0
                                        for i in session.xyDataObjects.keys():
                                            forceG12=forceG12+(session.xyDataObjects[i][0][1])

                                        stressG12 = abs(forceG12/(L*Thikness))

                                        G12 = stressG12/((Dispx/H)+(Dispy/L))

                                ## Shear modulus G13 ##                                
                                if G12==False or onlyPBC == True:
                                        G12='N/A'


                                E33 ='N/A'
                                G13 ='N/A'
                                G23 ='N/A'
                                
                                density = 0
                                if mass != None:
                                        density = mass/(L*W*H)
                                
                                print ('----------------------------------------------------')
                                print ('----------------------------------------------------')
                                print ('The homogenised elastic properties:')
                                print ('E11=%s Stress units' % (E11))
                                print ('V12=%s ratio' % (V12))
                                print ('E22=%s Stress units' % (E22))
                                print ('V21=%s ratio' % (V21))
                                print ('G12=%s Stress units' % (G12))
                                print ('----------------------------------------------------')
                                print ('Total mass=%s Mass units' % (mass))
                                print ('Homogenised density=%s Density units' % (density))
                                print ('----------------------------------------------------')
                                print ('Processing duration %s seconds' % (time.time()-start))
                                print ('----------------------------------------------------')
                                
                                filename = ('%s_elastic_properties.txt' % part)
                                print ('The homogenised elastic properties are saved in ABAQUS Work Directory under %s' % filename)
                                f = open(filename,'w')
                                f.write('{0:^10}{1:^20}{2:^20}\n'.format('Property','Value','Unit'))
                                f.write('{0:^10}{1:^20}{2:^20}\n'.format('E11',E11,'Stress units'))
                                f.write('{0:^10}{1:^20}{2:^20}\n'.format('V12',V12,'ratio'))
                                f.write('{0:^10}{1:^20}{2:^20}\n'.format('E22',E22,'Stress units'))
                                f.write('{0:^10}{1:^20}{2:^20}\n'.format('V21',V21,'ratio'))
                                f.write('{0:^10}{1:^20}{2:^20}\n'.format('G12',G12,'Stress units'))

                                f.write ('Total mass=%s Mass units \n' % (mass))
                                f.write ('Homogenised density=%s Density units \n' % (density))
                           
                                f.write ('processing duration %s Seconds' % (time.time()-start))

                                f.close()

                                print ('Citation: Omairey S, Dunning P, Sriramula S (2018) Development of an ABAQUS plugin tool for periodic RVE homogenisation.')
                                print ('Engineering with Computers. https://doi.org/10.1007/s00366-018-0616-4')


                                filename = ('%s_elastic_properties(easycopy).txt' % part)
                                f = open(filename,'w')
                                f.write('{0:^10}\n'.format(E11))
                                f.write('{0:^10}\n'.format(E22))
                                f.write('{0:^10}\n'.format(G12))
                                f.write('{0:^10}\n'.format(V12))
                                f.write('{0:^10}\n'.format(V21))
                                f.write ('{0:^10}\n' .format(mass))
                                f.write ('{0:^10}\n' .format(density))                   
                                f.write ('{0:^10}\n' .format((time.time()-start)))

                                f.close()

                                print ('----------------------------------------------------')
                                if onlyPBC == True:
                                        print ('EasyPBC created Period Boundary Conditions only. For further investigation, used relevant Reference Points to apply loads/displacements based on your needs. Details on the use of Reference Points are illustrated in Table 1 of the referred paper.')

                                        
                                
                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]
                                print ('---------------------------------------')
                                print ('--------- End of EasyPBC (2D) ---------')
                                print ('---------------------------------------')

                                if len(session.odbData.keys()) >= 1:                              
                                        odb.close(odb, write=TRUE)
                                        a = mdb.models[modelName].rootAssembly
                                        session.viewports['Viewport: 1'].setValues(displayedObject=a)

                        continue


                ## 3D model ##########################################################
                
                L=abs(Max-Mnx)
                H=abs(May-Mny)
                W=abs(Maz-Mnz)
                
                Dispx = L*0.1
                Dispy = H*0.1
                Dispz = W*0.1
                
                ## Creating Ref. Points ##
                for i in a.features.keys():
                    if i.startswith('RP'):
                        del a.features['%s' % (i)]
                a.ReferencePoint(point=(Max+0.8*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP6: G23
                a.ReferencePoint(point=(Max+0.6*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP5: G13
                a.ReferencePoint(point=(Max+0.4*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP4: G12
                a.ReferencePoint(point=(Max+0.2*abs(Max-Mnx), May-0.5*(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP3: Rigid body movement X-axis
                a.ReferencePoint(point=(Max-0.5*(Max-Mnx), May-0.5*(May-Mny), Maz+0.2*abs(Maz-Mnz)))  ## RP2: Rigid body movement Z-axis
                a.ReferencePoint(point=(Max-0.5*(Max-Mnx), May+0.2*abs(May-Mny), Maz-0.5*(Maz-Mnz)))  ## RP1: Rigid body movement Y-axis

                r1 = a.referencePoints

                ## Naming Ref. Points ##
                d=1
                for i in r1.keys():
                    refPoints1=(r1[i], )
                    if a.sets.has_key('RP%s' % (d)):
                            del a.sets['RP%s' % (d)]
                    a.Set(referencePoints=refPoints1, name='RP%s' % (d))
                    d=d+1
                  
                ## Identifying boundary nodes ##
                for i in Nodeset:
                    if (Mnx+meshsens) < i.coordinates[0] < (Max-meshsens) and (Mny+meshsens) < i.coordinates[1] < (May-meshsens) and (Mnz+meshsens) < i.coordinates[2] < (Maz-meshsens):
                        continue
                    if abs(i.coordinates[0]-Max)<=meshsens:
                        frontbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[0]-Mnx)<=meshsens:
                        backbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[2]-Maz)<=meshsens:
                        leftbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[2]-Mnz)<=meshsens:
                        rightbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[1]-May)<=meshsens:
                        topbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[1]-Mny)<=meshsens:
                        botbcxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens:
                        c1.insert(0,i.label)
                        coc1[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens:
                        c2.insert(0,i.label)
                        coc2[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens:
                        c3.insert(0,i.label)
                        coc3[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens:
                        c4.insert(0,i.label)
                        coc4[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens:
                        c5.insert(0,i.label)
                        coc5[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens:
                        c6.insert(0,i.label)
                        coc6[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens:
                        c7.insert(0,i.label)
                        coc7[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens:
                        c8.insert(0,i.label)
                        coc8[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                        ftedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                        fbedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                        btedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                        bbedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                        fledgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                        fredgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                        bledgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens:
                        bredgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                        ltedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                        lbedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                        rtedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                        rbedgexyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[0]-Max)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                        frontsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[0]-Mnx)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                        backsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]] 
                    if abs(i.coordinates[2]-Maz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                        leftsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]	
                    if abs(i.coordinates[2]-Mnz)<=meshsens and abs(i.coordinates[1]-May)>meshsens and abs(i.coordinates[1]-Mny)>meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens:
                        rightsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]   
                    if abs(i.coordinates[1]-May)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                        topsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]
                    if abs(i.coordinates[1]-Mny)<=meshsens and abs(i.coordinates[0]-Max)>meshsens and abs(i.coordinates[0]-Mnx)>meshsens and abs(i.coordinates[2]-Maz)>meshsens and abs(i.coordinates[2]-Mnz)>meshsens:
                        botsxyz[i.label]=[i.coordinates[0], i.coordinates[1], i.coordinates[2]]

                ## Checking number of nodes of opposite/associated sets ##
                if len(frontsxyz) != len(backsxyz):
                 print('Warning: Number of Nodes in Front surface (fronts) not equal to number of nodes in Back surface (backs). These sets will not be created!!')
                 print('         Refer to error 06 troubleshooting in easyPBC user guide.')
                 frontsxyz={}
                 error=True
                if len(topsxyz) != len(botsxyz):
                 print('Warning: Number of Nodes in Top surface (tops) not equal to number of nodes in Bottom surface (bots). These sets will not be created!!')
                 print('         Refer to error 06 in easyPBC user guide.')
                 topsxyz={}
                 error=True
                if len(leftsxyz) != len(rightsxyz):
                 print('Warning: Number of Nodes in Left surface (lefts) not equal to number of nodes in Right surface (rights). These sets will not be created!!')
                 print('         Refer to error 06 in easyPBC user guide.')
                 leftsxyz={}
                 error=True
                if len(ftedgexyz) != len(btedgexyz) or len(btedgexyz) != len(bbedgexyz) or len(bbedgexyz) != len(ftedgexyz):
                 print('Warning: Number of nodes in front-top ,back-top, back-bottom, front-bottom (ftedge, btedge, bbedge and fbedge) are not equal. These sets will not be created!!')
                 print('         Refer to error 06 in easyPBC user guide.')
                 ftedgexyz={}
                 error=True
                if len(fledgexyz) != len(bledgexyz) or len(bledgexyz) != len(bredgexyz) or len(bredgexyz) != len(fredgexyz):
                 print('Warning: Number of nodes in front-left, back-left, back-right, front-right edge (fledge, bledge, bredge and fredge) are not equal. These sets will not be created!!')
                 print('         Refer to error 06 in easyPBC user guide.')
                 fledgexyz={}
                 error=True
                if len(ltedgexyz) != len(rtedgexyz) or len(rtedgexyz) != len(rbedgexyz) or len(rbedgexyz) != len(lbedgexyz):
                 print('Warning: Number of nodes in left-top, right-top, right-bottom, front-bottom edge (ltedge, rtedge, rbedge and fbedge). are not equal. These sets will not be created!!')
                 print('         Refer to error 06 in easyPBC user guide.')
                 ltedgexyz={}
                 error=True
                if len(frontbcxyz) != len(backbcxyz):
                 print('Warning: Number of Nodes in Front BC surface (frontbc) not equal to number of nodes in Back BC surface (backbc). These sets will not be created!!')
                 print('         Refer to error 06 troubleshooting in easyPBC user guide.')
                 frontbcxyz={}
                 error=True
                if len(topbcxyz) != len(botbcxyz):
                 print('Warning: Number of Nodes in Top BC surface (topbc) not equal to number of nodes in Bottom BC surface (botbc). These sets will not be created!!')
                 print('         Refer to error 06 in easyPBC user guide.')
                 topbcxyz={}
                 error=True
                if len(leftbcxyz) != len(rightbcxyz):
                 print('Warning: Number of Nodes in Left BC surface (leftbc) not equal to number of nodes in Right BC surface (rightbc). These sets will not be created!!')
                 print('         Refer to error 06 in easyPBC user guide.')
                 leftbcxyz={}
                 error=True

                ## Sorting and appending sets ##
                for i in frontsxyz.keys():
                        for k in backsxyz.keys():
                                if abs(frontsxyz[i][1] - backsxyz[k][1])<=meshsens and abs(frontsxyz[i][2] - backsxyz[k][2])<=meshsens:
                                        fronts.append(i)
                                        backs.append(k)

                if len(frontsxyz)!= len(fronts) or len(backsxyz)!= len(backs):
                    print('Warning: Node(s) in Front and/or Back surface (fronts and/or backs) was not imported. effected sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                    for i, k in zip(frontsxyz.keys(),backsxyz.keys()):
                            if i not in fronts:
                                    errorset.append(i)
                            if k not in backs:
                                    errorset.append(k)
                    fronts=[]
                    backs=[]
                    error=True                    
                if len(fronts)!=len(set(fronts)) or len(backs)!=len(set(backs)):
                    print('Warning: Node(s) in either Front or Back surface (fronts or backs) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    fronts=[]
                    backs=[]
                    error=True

                for i in topsxyz.keys():
                    for k in botsxyz.keys():
                        if abs(topsxyz[i][0] - botsxyz[k][0]) <=meshsens and abs(topsxyz[i][2] - botsxyz[k][2]) <=meshsens:
                            tops.append(i)
                            bots.append(k)
                if len(topsxyz)!= len(tops) or len(botsxyz)!= len(bots):
                    print('Warning: Node(s) in Top and/or Bottom surface (tops and/or bots) was not imported. effected sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                    for i, k in zip(topsxyz.keys(),botsxyz.keys()):
                            if i not in tops:
                                    errorset.append(i)
                            if k not in bots:
                                    errorset.append(k)
                    tops=[]
                    bots=[]
                    error=True
                if len(tops)!=len(set(tops)) or len(bots)!=len(set(bots)):
                    print('Warning: Node(s) in either Top or Bottom surface (tops or bots) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    tops=[]
                    bots=[]
                    error=True


                for i in leftsxyz.keys():
                    for k in rightsxyz.keys():
                        if abs(leftsxyz[i][0] - rightsxyz[k][0])<=meshsens and abs(leftsxyz[i][1] - rightsxyz[k][1]) <=meshsens:
                            lefts.append(i)
                            rights.append(k)
                if len(leftsxyz)!= len(lefts) or len(rightsxyz)!= len(rights):
                    print('Warning: Node(s) in Left and/or Right surface (lefts and/or rights) was not imported. effected sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                    for i, k in zip(leftsxyz.keys(),rightsxyz.keys()):
                            if i not in lefts:
                                    errorset.append(i)
                            if k not in rights:
                                    errorset.append(k)                    
                    lefts=[]
                    rights=[]
                    error=True                    
                if len(lefts)!=len(set(lefts)) or len(rights)!=len(set(rights)):
                    print('Warning: Node(s) in either Left or Right surface (lefts or rights) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    lefts=[]
                    rights=[]
                    error=True

                for i in frontbcxyz.keys():
                    for k in backbcxyz.keys():
                        if abs(frontbcxyz[i][1] - backbcxyz[k][1])<=meshsens and abs(frontbcxyz[i][2] - backbcxyz[k][2])<=meshsens:
                            frontbc.append(i)
                            backbc.append(k)
                if len(frontbcxyz)!= len(frontbc) or len(backbcxyz)!= len(backbc):
                    print('Warning: Node(s) in Front BC and/or Back BC surface (frontbc and/or backbc) was not imported. effected sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                    for i, k in zip(frontbcxyz.keys(),backbcxyz.keys()):
                            if i not in frontbc:
                                    errorset.append(i)
                            if k not in backbc:
                                    errorset.append(k)
                    frontbc=[]
                    backbc=[]
                    error=True
                if len(frontbc)!=len(set(frontbc)) or len(backbc)!=len(set(backbc)):
                    print('Warning: Node(s) in either Front BC or Back BC surface (frontbc or backbc) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    frontbc=[]
                    backbc=[]
                    error=True


                for i in topbcxyz.keys():
                    for k in botbcxyz.keys():
                        if abs(topbcxyz[i][0] - botbcxyz[k][0]) <=meshsens and abs(topbcxyz[i][2] - botbcxyz[k][2]) <=meshsens:
                            topbc.append(i)
                            botbc.append(k)
                if len(topbcxyz)!= len(topbc) or len(botbcxyz)!= len(botbc):
                    print('Warning: Node(s) in Top BC and/or Bottom BC surface (topbc and/or botbc) was not imported. effected sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                    for i, k in zip(topbcxyz.keys(),botbcxyz.keys()):
                            if i not in topbc:
                                    errorset.append(i)
                            if k not in botbc:
                                    errorset.append(k)
                    topbc=[]
                    botbc=[]
                    error=True
                if len(topbc)!=len(set(topbc)) or len(botbc)!=len(set(botbc)):
                    print('Warning: Node(s) in either Top BC or Bottom BC surface (topbc or botbc) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    topbc=[]
                    botbc=[]
                    error=True


                for i in leftbcxyz.keys():
                    for k in rightbcxyz.keys():
                        if abs(leftbcxyz[i][0] - rightbcxyz[k][0])<=meshsens and abs(leftbcxyz[i][1] - rightbcxyz[k][1]) <=meshsens:
                            leftbc.append(i)
                            rightbc.append(k)
                if len(leftbcxyz)!= len(leftbc) or len(rightbcxyz)!= len(rightbc):
                    print('Warning: Node(s) in Left BC and/or Right BC surface (lefts and/or rights) was not imported. effected sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                    for i, k in zip(leftbcxyz.keys(),rightbcxyz.keys()):
                            if i not in leftbc:
                                    errorset.append(i)
                            if k not in rightbc:
                                    errorset.append(k)            
                    leftbc=[]
                    rightbc=[]
                    error=True
                if len(leftbc)!=len(set(leftbc)) or len(rightbc)!=len(set(rightbc)):
                    print('Warning: Node(s) in either Left BC or Right BC surface (leftbc or rightbc) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    leftbc=[]
                    rightbc=[]
                    error=True


                for i in ftedgexyz.keys():
                    for k in btedgexyz.keys():
                        if abs(ftedgexyz[i][1] - btedgexyz[k][1])<=meshsens and abs(ftedgexyz[i][2] - btedgexyz[k][2])<=meshsens:
                            ftedge.append(i)
                            btedge.append(k)
                for i in btedge:
                    for k in bbedgexyz.keys():
                        if abs(btedgexyz[i][0] - bbedgexyz[k][0]) <=meshsens and abs(btedgexyz[i][2] - bbedgexyz[k][2]) <=meshsens:
                            bbedge.append(k)    
                for i in bbedge:
                    for k in fbedgexyz.keys():
                        if abs(bbedgexyz[i][1] - fbedgexyz[k][1]) <=meshsens and abs(bbedgexyz[i][2] - fbedgexyz[k][2]) <=meshsens:
                            fbedge.append(k) 
                if len(ftedge)!=len(set(ftedge)) or len(btedge)!=len(set(btedge)) or len(bbedge)!=len(set(bbedge)) or len(fbedge)!=len(set(fbedge)):
                    print('Warning: Node(s) in either front-top, back-top, back-bottom and front-bottom edge(ftedge, btedge, bbedge and fbedge) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    ftedge=[]
                    btedge=[]
                    bbedg=[]
                    fbedge=[]
                    error==True
                if len(ftedgexyz)!= len(ftedge) or len(btedgexyz)!= len(btedge) or len(bbedgexyz)!= len(bbedge) or len(fbedgexyz)!= len(fbedge):
                    print('Warning: Node(s) in front-top, back-top, back-bottom and front-bottom edge(ftedge, btedge, bbedge and fbedge) were not imported. these sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                    ftedge=[]
                    btedge=[]
                    bbedg=[]
                    fbedge=[]
                    error=True

                for i in ltedgexyz.keys():
                    for k in rtedgexyz.keys():
                        if abs(ltedgexyz[i][0] - rtedgexyz[k][0])<=meshsens and abs(ltedgexyz[i][1] - rtedgexyz[k][1])<=meshsens:
                            ltedge.append(i)
                            rtedge.append(k)
                for i in rtedge:
                    for k in rbedgexyz.keys():
                        if abs(rtedgexyz[i][0] - rbedgexyz[k][0])<=meshsens and abs(rtedgexyz[i][2] - rbedgexyz[k][2])<=meshsens:
                            rbedge.append(k)    
                for i in rbedge:
                    for k in lbedgexyz.keys():
                        if abs(rbedgexyz[i][0] - lbedgexyz[k][0])<=meshsens and abs(rbedgexyz[i][1] - lbedgexyz[k][1])<=meshsens:
                            lbedge.append(k) 

                if len(ltedge)!=len(set(ltedge)) or len(rtedge)!=len(set(rtedge)) or len(rbedge)!=len(set(rbedge)) or len(lbedge)!=len(set(lbedge)):
                    print('Warning: Node(s) in either front-top, back-bottom and front-bottom edge(ltedge, rtedge, rbedge and lbedge) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    ltedge=[]
                    rtedge=[]
                    rbedg=[]
                    lbedge=[]
                    error=True

                if len(ltedgexyz)!= len(ltedge) or len(rtedgexyz)!= len(rtedge) or len(rbedgexyz)!= len(rbedge) or len(lbedgexyz)!= len(lbedge):
                    print('Warning: Node(s) in left-top, right-top, right-bottom, left-bottom edge (ltedge, rtedge, rbedge and lbedge) were not imported. these sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user guide and created Error set (if applicable).')
                    ltedge=[]
                    rtedge=[]
                    rbedg=[]
                    lbedge=[]
                    error=True

                for i in fledgexyz.keys():
                    for k in bledgexyz.keys():
                        if abs(fledgexyz[i][1] - bledgexyz[k][1])<=meshsens and abs(fledgexyz[i][2] - bledgexyz[k][2])<=meshsens:
                            fledge.append(i)
                            bledge.append(k)
                for i in bledge:
                    for k in bredgexyz.keys():
                        if abs(bledgexyz[i][0] - bredgexyz[k][0])<=meshsens and abs(bledgexyz[i][1] - bredgexyz[k][1])<=meshsens:
                            bredge.append(k)    
                for i in bredge:
                    for k in fredgexyz.keys():
                        if abs(bredgexyz[i][1] - fredgexyz[k][1])<=meshsens and abs(bredgexyz[i][2] - fredgexyz[k][2])<=meshsens:
                            fredge.append(k) 

                if len(fledge)!=len(set(fledge)) or len(bledge)!=len(set(bledge)) or len(bredge)!=len(set(bredge)) or len(fredge)!=len(set(fredge)):
                    print('Warning: Node(s) in either front-left, back-left, back-right and front-right edge(fledge, bledge, bredge and fredge) being linked with more than one opposite node. effected sets will not be created!!')
                    print('         Refer to error 08 in easyPBC user guide.')
                    fledge=[]
                    bledge=[]
                    bredg=[]
                    fredge=[]
                    error=True
                if len(fledgexyz)!= len(fledge) or len(bledgexyz)!= len(bledge) or len(bredgexyz)!= len(bredge) or len(fredgexyz)!= len(fredge):
                    print('Warning: Node(s) in front-left, back-left, back-right and front-right edge (fledge, bledge, bredge and fredge) were not imported. these sets will not be created!!')
                    print('         Refer to error 07 in easyPBC user user guide.')
                    fledge=[]
                    bledge=[]
                    bredg=[]
                    fredge=[]
                    error=True

                if (reusePBC and pbc_exists):
                        print ('------ EasyPBC sets already exist: skip sets creation ------')
                else:
                        ## Creating ABAQUS sets ##
                        a.SetFromNodeLabels(name='c1', nodeLabels=((instanceName,c1),))
                        a.SetFromNodeLabels(name='c2', nodeLabels=((instanceName,c2),))
                        a.SetFromNodeLabels(name='c3', nodeLabels=((instanceName,c3),))
                        a.SetFromNodeLabels(name='c4', nodeLabels=((instanceName,c4),))
                        a.SetFromNodeLabels(name='c5', nodeLabels=((instanceName,c5),))
                        a.SetFromNodeLabels(name='c6', nodeLabels=((instanceName,c6),))
                        a.SetFromNodeLabels(name='c7', nodeLabels=((instanceName,c7),))
                        a.SetFromNodeLabels(name='c8', nodeLabels=((instanceName,c8),))
                        a.SetFromNodeLabels(name='ftedge', nodeLabels=((instanceName,ftedge),))
                        a.SetFromNodeLabels(name='fbedge', nodeLabels=((instanceName,fbedge),))
                        a.SetFromNodeLabels(name='btedge', nodeLabels=((instanceName,btedge),))
                        a.SetFromNodeLabels(name='bbedge', nodeLabels=((instanceName,bbedge),))
                        a.SetFromNodeLabels(name='fledge', nodeLabels=((instanceName,fledge),))
                        a.SetFromNodeLabels(name='fredge', nodeLabels=((instanceName,fredge),))
                        a.SetFromNodeLabels(name='bledge', nodeLabels=((instanceName,bledge),))
                        a.SetFromNodeLabels(name='bredge', nodeLabels=((instanceName,bredge),))
                        a.SetFromNodeLabels(name='ltedge', nodeLabels=((instanceName,ltedge),))
                        a.SetFromNodeLabels(name='lbedge', nodeLabels=((instanceName,lbedge),))
                        a.SetFromNodeLabels(name='rtedge', nodeLabels=((instanceName,rtedge),))
                        a.SetFromNodeLabels(name='rbedge', nodeLabels=((instanceName,rbedge),))
                        a.SetFromNodeLabels(name='fronts', nodeLabels=((instanceName,fronts),))
                        a.SetFromNodeLabels(name='backs', nodeLabels=((instanceName,backs),))
                        a.SetFromNodeLabels(name='lefts', nodeLabels=((instanceName,lefts),))
                        a.SetFromNodeLabels(name='rights', nodeLabels=((instanceName,rights),))
                        a.SetFromNodeLabels(name='tops', nodeLabels=((instanceName,tops),))
                        a.SetFromNodeLabels(name='bots', nodeLabels=((instanceName,bots),))
                        a.SetFromNodeLabels(name='frontbc', nodeLabels=((instanceName,frontbc),))
                        a.SetFromNodeLabels(name='backbc', nodeLabels=((instanceName,backbc),))
                        a.SetFromNodeLabels(name='leftbc', nodeLabels=((instanceName,leftbc),))
                        a.SetFromNodeLabels(name='rightbc', nodeLabels=((instanceName,rightbc),))
                        a.SetFromNodeLabels(name='topbc', nodeLabels=((instanceName,topbc),))
                        a.SetFromNodeLabels(name='botbc', nodeLabels=((instanceName,botbc),))
                        print ('------ End of Sets Creation ------')

                ## Extracting model mass ##
                prop=mdb.models[modelName].rootAssembly.getMassProperties()
                mass=prop['mass']

                a = mdb.models[modelName].rootAssembly
                Nodeset = mdb.models[modelName].rootAssembly.instances[instanceName].nodes
                if not mdb.models[modelName].steps.has_key('Step-1'):
                        mdb.models[modelName].StaticStep(name='Step-1', previous='Initial')
                # Ensure IVOL is available for strict volume-weighted post-processing
                if requestIVOL:
                        try:
                                _fo_name = 'F-IVOL'
                                _reqs = mdb.models[modelName].fieldOutputRequests
                                if _fo_name in _reqs.keys():
                                        try:
                                                _reqs[_fo_name].setValues(variables=('IVOL',))
                                        except:
                                                pass
                                else:
                                        mdb.models[modelName].FieldOutputRequest(name=_fo_name, createStepName='Step-1', variables=('IVOL',))
                        except Exception as _e:
                                # Fallback: try to append IVOL to 'F-Output-1' (handles PRESELECT safely)
                                try:
                                        if 'F-Output-1' in mdb.models[modelName].fieldOutputRequests.keys():
                                                _fo = mdb.models[modelName].fieldOutputRequests['F-Output-1']
                                                _v0 = getattr(_fo, 'variables', None)
                                                if _v0 == PRESELECT:
                                                        _fo.setValues(variables=('S','E','LE','U','RF','IVOL'))
                                                else:
                                                        try:
                                                                _vlist = list(_v0) if hasattr(_v0, '__iter__') else []
                                                        except:
                                                                _vlist = []
                                                        if 'IVOL' not in _vlist:
                                                                _vlist.append('IVOL')
                                                        try:
                                                                _fo.setValues(variables=tuple(_vlist))
                                                        except:
                                                                pass
                                        else:
                                                mdb.models[modelName].FieldOutputRequest(name='F-Output-1', createStepName='Step-1', variables=('S','E','LE','U','RF','IVOL'))
                                except:
                                        pass

                ## Creating single-node ABAQUS sets ##                
                if error==False:
                        
                        if (reusePBC and pbc_exists):
                                # Single-node sets already exist from a previous run
                                pass
                        else:
                                if E11==True or E22==True or E33==True or G12==True or G13==True or G23==True:
                                        for i,k in zip(tops,bots):
                                            a.SetFromNodeLabels(name='tops%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='bots%s' % (k), nodeLabels=((instanceName,[k]),))
        
                                        for i,k in zip(fronts,backs):
                                            a.SetFromNodeLabels(name='fronts%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='backs%s' % (k), nodeLabels=((instanceName,[k]),))
        
                                        for i,k in zip(lefts,rights):
                                            a.SetFromNodeLabels(name='lefts%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='rights%s' % (k), nodeLabels=((instanceName,[k]),))
        
                                        for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                                            a.SetFromNodeLabels(name='ftedge%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='btedge%s' % (k), nodeLabels=((instanceName,[k]),))
                                            a.SetFromNodeLabels(name='bbedge%s' % (j), nodeLabels=((instanceName,[j]),))
                                            a.SetFromNodeLabels(name='fbedge%s' % (l), nodeLabels=((instanceName,[l]),))
        
                                        for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                                            a.SetFromNodeLabels(name='fledge%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='bledge%s' % (k), nodeLabels=((instanceName,[k]),))
                                            a.SetFromNodeLabels(name='bredge%s' % (j), nodeLabels=((instanceName,[j]),))
                                            a.SetFromNodeLabels(name='fredge%s' % (l), nodeLabels=((instanceName,[l]),))
        
                                        for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                                            a.SetFromNodeLabels(name='ltedge%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='lbedge%s' % (k), nodeLabels=((instanceName,[k]),))
                                            a.SetFromNodeLabels(name='rbedge%s' % (j), nodeLabels=((instanceName,[j]),))
                                            a.SetFromNodeLabels(name='rtedge%s' % (l), nodeLabels=((instanceName,[l]),))
        
                                        for i,k in zip(topbc,botbc):
                                            a.SetFromNodeLabels(name='topbc%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='botbc%s' % (k), nodeLabels=((instanceName,[k]),))
        
                                        for i,k in zip(frontbc,backbc):
                                            a.SetFromNodeLabels(name='frontbc%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='backbc%s' % (k), nodeLabels=((instanceName,[k]),))
        
                                        for i,k in zip(leftbc,rightbc):
                                            a.SetFromNodeLabels(name='leftbc%s' % (i), nodeLabels=((instanceName,[i]),))
                                            a.SetFromNodeLabels(name='rightbc%s' % (k), nodeLabels=((instanceName,[k]),))
        
                        ## Creating constraints for elastic moduli ##
                        if E11==True or E22==True or E33==True:
                                for i in mdb.models[modelName].constraints.keys():
                                        del mdb.models[modelName].constraints[i]
                                        
                                for i,k in zip(tops,bots):
                                    mdb.models[modelName].Equation(name='E-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1)))
                                for i,k in zip(tops,bots):
                                    mdb.models[modelName].Equation(name='E-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2),(-1.0, 'RP5', 2)))
                                for i,k in zip(tops,bots):
                                    mdb.models[modelName].Equation(name='E-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3)))

                                for i,k in zip(lefts,rights):
                                    mdb.models[modelName].Equation(name='E-1-lefts-rights%s'%i, terms=((1.0, 'lefts%s'%i, 1), (-1.0, 'rights%s'%k, 1)))
                                for i,k in zip(lefts,rights):
                                    mdb.models[modelName].Equation(name='E-2-lefts-rights%s'%i, terms=((1.0, 'lefts%s'%i, 2), (-1.0, 'rights%s'%k, 2)))
                                for i,k in zip(lefts,rights):
                                    mdb.models[modelName].Equation(name='E-3-lefts-rights%s'%i, terms=((1.0, 'lefts%s'%i, 3), (-1.0, 'rights%s'%k, 3),(-1.0, 'RP6', 3)))

                                for i,k in zip(fronts,backs):
                                    mdb.models[modelName].Equation(name='E-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1),(-1.0, 'RP4', 1)))
                                for i,k in zip(fronts,backs):
                                    mdb.models[modelName].Equation(name='E-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2)))
                                for i,k in zip(fronts,backs):
                                    mdb.models[modelName].Equation(name='E-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3)))


                                mdb.models[modelName].Equation(name='E-1-c12', terms=((1.0, 'c6', 1), (-1.0, 'c2', 1)))
                                mdb.models[modelName].Equation(name='E-1-c23', terms=((1.0, 'c2', 1), (-1.0, 'c3', 1)))
                                mdb.models[modelName].Equation(name='E-1-c34', terms=((1.0, 'c3', 1), (-1.0, 'c4', 1),(1.0, 'RP4', 1)))
                                mdb.models[modelName].Equation(name='E-1-c45', terms=((1.0, 'c4', 1), (-1.0, 'c8', 1)))
                                mdb.models[modelName].Equation(name='E-1-c56', terms=((1.0, 'c8', 1), (-1.0, 'c5', 1)))
                                mdb.models[modelName].Equation(name='E-1-c67', terms=((1.0, 'c5', 1), (-1.0, 'c1', 1)))
                                mdb.models[modelName].Equation(name='E-1-c78', terms=((1.0, 'c1', 1), (-1.0, 'c7', 1),(-1.0, 'RP4', 1)))
                               
                                mdb.models[modelName].Equation(name='E-2-c12', terms=((1.0, 'c6', 2), (-1.0, 'c2', 2),(1.0, 'RP5', 2)))
                                mdb.models[modelName].Equation(name='E-2-c23', terms=((1.0, 'c2', 2), (-1.0, 'c3', 2)))
                                mdb.models[modelName].Equation(name='E-2-c34', terms=((1.0, 'c3', 2), (-1.0, 'c4', 2)))
                                mdb.models[modelName].Equation(name='E-2-c45', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2),(-1.0, 'RP5', 2)))
                                mdb.models[modelName].Equation(name='E-2-c56', terms=((1.0, 'c8', 2), (-1.0, 'c5', 2)))
                                mdb.models[modelName].Equation(name='E-2-c67', terms=((1.0, 'c5', 2), (-1.0, 'c1', 2),(1.0, 'RP5', 2)))
                                mdb.models[modelName].Equation(name='E-2-c78', terms=((1.0, 'c1', 2), (-1.0, 'c7', 2),(-1.0, 'RP5', 2)))

                                mdb.models[modelName].Equation(name='E-3-c12', terms=((1.0, 'c6', 3), (-1.0, 'c2', 3)))
                                mdb.models[modelName].Equation(name='E-3-c23', terms=((1.0, 'c2', 3), (-1.0, 'c3', 3),(-1.0, 'RP6', 3)))
                                mdb.models[modelName].Equation(name='E-3-c34', terms=((1.0, 'c3', 3), (-1.0, 'c4', 3)))
                                mdb.models[modelName].Equation(name='E-3-c45', terms=((1.0, 'c4', 3), (-1.0, 'c8', 3)))
                                mdb.models[modelName].Equation(name='E-3-c56', terms=((1.0, 'c8', 3), (-1.0, 'c5', 3),(1.0, 'RP6', 3)))
                                mdb.models[modelName].Equation(name='E-3-c67', terms=((1.0, 'c5', 3), (-1.0, 'c1', 3)))
                                mdb.models[modelName].Equation(name='E-3-c78', terms=((1.0, 'c1', 3), (-1.0, 'c7', 3),(-1.0, 'RP6', 3)))
                                       

                                for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                                    mdb.models[modelName].Equation(name='E-1-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 1), (-1.0, 'btedge%s'%k, 1),(-1.0, 'RP4', 1)))
                                    mdb.models[modelName].Equation(name='E-1-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 1), (-1.0, 'bbedge%s'%j, 1)))
                                    mdb.models[modelName].Equation(name='E-1-bbedge-fbedge%s'%j, terms=((1.0, 'bbedge%s'%j, 1), (-1.0, 'fbedge%s'%l, 1),(1.0, 'RP4', 1)))
                                for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                                    mdb.models[modelName].Equation(name='E-2-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 2), (-1.0, 'btedge%s'%k, 2)))
                                    mdb.models[modelName].Equation(name='E-2-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 2), (-1.0, 'bbedge%s'%j, 2),(-1.0, 'RP5', 2)))
                                    mdb.models[modelName].Equation(name='E-2-bbedge-fbedge%s'%j, terms=((1.0, 'bbedge%s'%j, 2), (-1.0, 'fbedge%s'%l, 2)))
                                for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                                    mdb.models[modelName].Equation(name='E-3-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'btedge%s'%k, 3)))
                                    mdb.models[modelName].Equation(name='E-3-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 3), (-1.0, 'bbedge%s'%j, 3)))
                                    mdb.models[modelName].Equation(name='E-3-bbedge-fbedge%s'%j, terms=((1.0, 'bbedge%s'%j, 3), (-1.0, 'fbedge%s'%l, 3)))

                                for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                                    mdb.models[modelName].Equation(name='E-1-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 1), (-1.0, 'bledge%s'%k, 1),(-1.0, 'RP4', 1)))
                                    mdb.models[modelName].Equation(name='E-1-bledge-bredge%s'%k, terms=((1.0, 'bledge%s'%k, 1), (-1.0, 'bredge%s'%j, 1)))
                                    mdb.models[modelName].Equation(name='E-1-bredge-fredge%s'%j, terms=((1.0, 'bredge%s'%j, 1), (-1.0, 'fredge%s'%l, 1),(1.0, 'RP4', 1)))
                                for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                                    mdb.models[modelName].Equation(name='E-2-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 2), (-1.0, 'bledge%s'%k, 2)))
                                    mdb.models[modelName].Equation(name='E-2-bledge-bredge%s'%k, terms=((1.0, 'bledge%s'%k, 2), (-1.0, 'bredge%s'%j, 2)))
                                    mdb.models[modelName].Equation(name='E-2-bredge-fredge%s'%j, terms=((1.0, 'bredge%s'%j, 2), (-1.0, 'fredge%s'%l, 2)))
                                for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                                    mdb.models[modelName].Equation(name='E-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3)))
                                    mdb.models[modelName].Equation(name='E-3-bledge-bredge%s'%k, terms=((1.0, 'bledge%s'%k, 3), (-1.0, 'bredge%s'%j, 3),(-1.0, 'RP6', 3)))
                                    mdb.models[modelName].Equation(name='E-3-bredge-fredge%s'%j, terms=((1.0, 'bredge%s'%j, 3), (-1.0, 'fredge%s'%l, 3)))

                                for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                                    mdb.models[modelName].Equation(name='E-1-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 1), (-1.0, 'lbedge%s'%k, 1)))
                                    mdb.models[modelName].Equation(name='E-1-lbtedge-rbedge%s'%k, terms=((1.0, 'lbedge%s'%k, 1), (-1.0, 'rbedge%s'%j, 1)))
                                    mdb.models[modelName].Equation(name='E-1-rbedge-rtbedge%s'%j, terms=((1.0, 'rbedge%s'%j, 1), (-1.0, 'rtedge%s'%l, 1)))                                    
                                for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                                    mdb.models[modelName].Equation(name='E-2-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 2), (-1.0, 'lbedge%s'%k, 2),(-1.0, 'RP5', 2)))
                                    mdb.models[modelName].Equation(name='E-2-lbtedge-rbedge%s'%k, terms=((1.0, 'lbedge%s'%k, 2), (-1.0, 'rbedge%s'%j, 2)))
                                    mdb.models[modelName].Equation(name='E-2-rbedge-rtbedge%s'%j, terms=((1.0, 'rbedge%s'%j, 2), (-1.0, 'rtedge%s'%l, 2),(1.0, 'RP5', 2)))
                                for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                                    mdb.models[modelName].Equation(name='E-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3)))
                                    mdb.models[modelName].Equation(name='E-3-lbtedge-rbedge%s'%k, terms=((1.0, 'lbedge%s'%k, 3), (-1.0, 'rbedge%s'%j, 3),(-1.0, 'RP6', 3)))
                                    mdb.models[modelName].Equation(name='E-3-rbedge-rtbedge%s'%j, terms=((1.0, 'rbedge%s'%j, 3), (-1.0, 'rtedge%s'%l, 3)))

                        ## Elastic modulus E11 ##
                        if E11==True and onlyPBC == False:
                                for i in mdb.models[modelName].loads.keys():
                                        del mdb.models[modelName].loads[i]
                                for i in mdb.models[modelName].boundaryConditions.keys():
                                        del mdb.models[modelName].boundaryConditions[i]


                                region = a.sets['RP4']
                                mdb.models[modelName].DisplacementBC(name='E11-1', createStepName='Step-1', 
                                    region=region, u1=Dispx, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)

                                regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                        del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                    createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                    sectionPoints=DEFAULT, rebar=EXCLUDE)

                                import os, glob

                                if mdb.jobs.has_key('job-E11'):
                                        del mdb.jobs['job-E11']
                                mdb.Job(name='job-E11', model= modelName, description='', type=ANALYSIS, 
                                    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                    scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                mdb.jobs['job-E11'].submit(consistencyChecking=OFF)
                                mdb.jobs['job-E11'].waitForCompletion()
                                o3 = session.openOdb(name='%s' % (path+'\job-E11.odb'))
                             
                                odb = session.odbs['%s' % (path+'\job-E11.odb')]

                                session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                odbName=session.viewports[session.currentViewportName].odbDisplay.name



                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                    NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP4', ))

                                forceE11 = 0
                                for i in session.xyDataObjects.keys():
                                    forceE11=forceE11+(session.xyDataObjects[i][0][1])

                                stressE11 = abs(forceE11/(H*W))


                                E11 = stressE11/(Dispx/L)                                


                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]
                                
                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U1'), )), ), nodeSets=('C1','C2', ))
                                
                                C1U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][0]
                                
                                C2U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c2[0])][0][1] + coc2[(c2[0])][0]
                                Dis = abs(C1U1new - C2U1new)

                                E11U1= abs(L - Dis)


                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]
                                    

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U2'), )), ), nodeSets=('C1','C5', ))
                                
                                C1U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][1]
                                
                                C5U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c5[0])][0][1] + coc5[(c5[0])][1]
                                Dis = abs(C1U2new - C5U2new)

                                E11U2= abs(H - Dis)

                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]


                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U3'), )), ), nodeSets=('C1','C4', ))
                                
                                C1U3new = session.xyDataObjects['U:U3 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][2]
                                
                                C4U3new = session.xyDataObjects['U:U3 PI: %s N: %s' % (upperName,c4[0])][0][1] + coc4[(c4[0])][2]
                                Dis = abs(C1U3new - C4U3new)

                                E11U3= abs(W - Dis)


                                V12=(E11U2/H)/(E11U1/L)
                                V13=(E11U3/W)/(E11U1/L)



                        ## Elastic modulus E22 ##                                
                        if E11==False or onlyPBC == True:
                                E11='N/A'
                                V12='N/A'
                                V13='N/A'

                        if E22==True and onlyPBC == False:

                                for i in mdb.models[modelName].loads.keys():
                                        del mdb.models[modelName].loads[i]
                                for i in mdb.models[modelName].boundaryConditions.keys():
                                        del mdb.models[modelName].boundaryConditions[i]

                                region = a.sets['RP5']
                                mdb.models[modelName].DisplacementBC(name='E22-1', createStepName='Step-1', 
                                    region=region, u1=UNSET, u2=Dispy, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)


                                regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                        del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                    createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                    sectionPoints=DEFAULT, rebar=EXCLUDE)

                                import os, glob

                                if mdb.jobs.has_key('job-E22'):
                                        del mdb.jobs['job-E22']
                                mdb.Job(name='job-E22', model= modelName, description='', type=ANALYSIS, 
                                    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                    scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                mdb.jobs['job-E22'].submit(consistencyChecking=OFF)
                                mdb.jobs['job-E22'].waitForCompletion()
                                o3 = session.openOdb(name='%s' % (path+'\job-E22.odb'))
                             
                                odb = session.odbs['%s' % (path+'\job-E22.odb')]

                                session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                odbName=session.viewports[session.currentViewportName].odbDisplay.name


                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                    NODAL, ((COMPONENT, 'RF2'), )), ), nodeSets=('RP5', ))

                                forceE22 = 0
                                for i in session.xyDataObjects.keys():
                                    forceE22=forceE22+(session.xyDataObjects[i][0][1])

                                stressE22 = abs(forceE22/(W*L))


                                E22 = stressE22/(Dispy/H)                                




                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]


                                
                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U1'), )), ), nodeSets=('C1','C2', ))
                                
                                C1U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][0]
                                
                                C2U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c2[0])][0][1] + coc2[(c2[0])][0]
                                Dis = abs(C1U1new - C2U1new)

                                E22U1= abs(L - Dis)


                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]
                                    

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U2'), )), ), nodeSets=('C1','C5', ))
                                
                                C1U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][1]
                                
                                C5U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c5[0])][0][1] + coc5[(c5[0])][1]
                                Dis = abs(C1U2new - C5U2new)

                                E22U2= abs(H - Dis)

                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]


                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U3'), )), ), nodeSets=('C1','C4', ))
                                
                                C1U3new = session.xyDataObjects['U:U3 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][2]
                                
                                C4U3new = session.xyDataObjects['U:U3 PI: %s N: %s' % (upperName,c4[0])][0][1] + coc4[(c4[0])][2]
                                Dis = abs(C1U3new - C4U3new)

                                E22U3= abs(W - Dis)


                                V21=(E22U1/L)/(E22U2/H)
                                V23=(E22U3/W)/(E22U2/H)


                        ## Elastic modulus E33 ##
                        if E22==False or onlyPBC == True:
                                E22='N/A'
                                V21='N/A'
                                V23='N/A'


                        if E33==True and onlyPBC == False:
                                for i in mdb.models[modelName].loads.keys():
                                        del mdb.models[modelName].loads[i]
                                for i in mdb.models[modelName].boundaryConditions.keys():
                                        del mdb.models[modelName].boundaryConditions[i]


                                region = a.sets['RP6']
                                mdb.models[modelName].DisplacementBC(name='E33-1', createStepName='Step-1', 
                                    region=region, u1=UNSET, u2=UNSET, u3=Dispz, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)

                                regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                        del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                    createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                    sectionPoints=DEFAULT, rebar=EXCLUDE)

                                import os, glob

                                if mdb.jobs.has_key('job-E33'):
                                        del mdb.jobs['job-E33']
                                mdb.Job(name='job-E33', model= modelName, description='', type=ANALYSIS, 
                                    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                    scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                mdb.jobs['job-E33'].submit(consistencyChecking=OFF)
                                mdb.jobs['job-E33'].waitForCompletion()
                                o3 = session.openOdb(name='%s' % (path+'\job-E33.odb'))
                             
                                odb = session.odbs['%s' % (path+'\job-E33.odb')]

                                session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                odbName=session.viewports[session.currentViewportName].odbDisplay.name

                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                    NODAL, ((COMPONENT, 'RF3'), )), ), nodeSets=('RP6', ))

                                forceE33 = 0
                                for i in session.xyDataObjects.keys():
                                    forceE33=forceE33+(session.xyDataObjects[i][0][1])

                                stressE33 = abs(forceE33/(H*L))


                                E33 = stressE33/(Dispz/W)                                

                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]


                                
                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U1'), )), ), nodeSets=('C1','C2', ))
                                
                                C1U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][0]
                                
                                C2U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c2[0])][0][1] + coc2[(c2[0])][0]
                                Dis = abs(C1U1new - C2U1new)

                                E33U1= abs(L - Dis)


                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]
                                    

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U2'), )), ), nodeSets=('C1','C5', ))
                                
                                C1U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][1]
                                
                                C5U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c5[0])][0][1] + coc5[(c5[0])][1]
                                Dis = abs(C1U2new - C5U2new)

                                E33U2= abs(H - Dis)

                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]


                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U3'), )), ), nodeSets=('C1','C4', ))
                                
                                C1U3new = session.xyDataObjects['U:U3 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][2]
                                
                                C4U3new = session.xyDataObjects['U:U3 PI: %s N: %s' % (upperName,c4[0])][0][1] + coc4[(c4[0])][2]
                                Dis = abs(C1U3new - C4U3new)

                                E33U3= abs(W - Dis)


                                V31=(E33U1/L)/(E33U3/W)
                                V32=(E33U2/H)/(E33U3/W)


                        if E33==False or onlyPBC == True:
                                E33='N/A'
                                V31='N/A'
                                V32='N/A'

                        ## Creating constraints for shear moduli ##
                        if G12==True or G13==True or G23==True:
                                if onlyPBC == False:
                                        for i in mdb.models[modelName].constraints.keys():
                                                del mdb.models[modelName].constraints[i]

                                for i,k in zip(tops,bots):
                                    mdb.models[modelName].Equation(name='G-1-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 1), (-1.0, 'bots%s'%k, 1),(-1.0, 'RP4', 1)))
                                for i,k in zip(tops,bots):
                                    mdb.models[modelName].Equation(name='G-2-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 2), (-1.0, 'bots%s'%k, 2),(-1.0, 'RP1', 2)))
                                for i,k in zip(tops,bots):
                                    mdb.models[modelName].Equation(name='G-3-tops-bots%s'%i, terms=((1.0, 'tops%s'%i, 3), (-1.0, 'bots%s'%k, 3),(-1.0, 'RP6', 3)))

                               
                                for i,k in zip(lefts,rights):
                                    mdb.models[modelName].Equation(name='G-1-lefts-rights%s'%i, terms=((1.0, 'lefts%s'%i, 1), (-1.0, 'rights%s'%k, 1),(-1.0, 'RP5', 1)))
                                for i,k in zip(lefts,rights):
                                    mdb.models[modelName].Equation(name='G-2-lefts-rights%s'%i, terms=((1.0, 'lefts%s'%i, 2), (-1.0, 'rights%s'%k, 2),(-1.0, 'RP6', 2)))
                                for i,k in zip(lefts,rights):
                                    mdb.models[modelName].Equation(name='G-3-lefts-rights%s'%i, terms=((1.0, 'lefts%s'%i, 3), (-1.0, 'rights%s'%k, 3),(-1.0, 'RP2', 3)))

                                for i,k in zip(fronts,backs):
                                    mdb.models[modelName].Equation(name='G-1-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 1), (-1.0, 'backs%s'%k, 1),(-1.0, 'RP3', 1)))
                                for i,k in zip(fronts,backs):
                                    mdb.models[modelName].Equation(name='G-2-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 2), (-1.0, 'backs%s'%k, 2),(-1.0, 'RP4', 2)))
                                for i,k in zip(fronts,backs):
                                    mdb.models[modelName].Equation(name='G-3-fronts-backs%s'%i, terms=((1.0, 'fronts%s'%i, 3), (-1.0, 'backs%s'%k, 3),(-1.0, 'RP5', 3)))


                                mdb.models[modelName].Equation(name='G-1-c12', terms=((1.0, 'c6', 1), (-1.0, 'c2', 1),(1.0, 'RP4', 1)))
                                mdb.models[modelName].Equation(name='G-1-c23', terms=((1.0, 'c2', 1), (-1.0, 'c3', 1),(-1.0, 'RP5', 1)))
                                mdb.models[modelName].Equation(name='G-1-c34', terms=((1.0, 'c3', 1), (-1.0, 'c4', 1),(1.0, 'RP3', 1)))
                                mdb.models[modelName].Equation(name='G-1-c45', terms=((1.0, 'c4', 1), (-1.0, 'c8', 1),(-1.0, 'RP4', 1)))
                                mdb.models[modelName].Equation(name='G-1-c56', terms=((1.0, 'c8', 1), (-1.0, 'c5', 1),(1.0, 'RP5', 1)))
                                mdb.models[modelName].Equation(name='G-1-c67', terms=((1.0, 'c5', 1), (-1.0, 'c1', 1),(1.0, 'RP4', 1)))
                                mdb.models[modelName].Equation(name='G-1-c78', terms=((1.0, 'c1', 1), (-1.0, 'c7', 1),(-1.0, 'RP3', 1),(-1.0, 'RP4', 1),(-1.0, 'RP5', 1)))

                                    
                                mdb.models[modelName].Equation(name='G-2-c12', terms=((1.0, 'c6', 2), (-1.0, 'c2', 2),(1.0, 'RP1', 2)))
                                mdb.models[modelName].Equation(name='G-2-c23', terms=((1.0, 'c2', 2), (-1.0, 'c3', 2),(-1.0, 'RP6', 2)))
                                mdb.models[modelName].Equation(name='G-2-c34', terms=((1.0, 'c3', 2), (-1.0, 'c4', 2),(1.0, 'RP4', 2)))
                                mdb.models[modelName].Equation(name='G-2-c45', terms=((1.0, 'c4', 2), (-1.0, 'c8', 2),(-1.0, 'RP1', 2)))
                                mdb.models[modelName].Equation(name='G-2-c56', terms=((1.0, 'c8', 2), (-1.0, 'c5', 2),(1.0, 'RP6', 2)))
                                mdb.models[modelName].Equation(name='G-2-c67', terms=((1.0, 'c5', 2), (-1.0, 'c1', 2),(1.0, 'RP1', 2)))
                                mdb.models[modelName].Equation(name='G-2-c78', terms=((1.0, 'c1', 2), (-1.0, 'c7', 2),(-1.0, 'RP1', 2),(-1.0, 'RP4', 2),(-1.0, 'RP6', 2)))

        
                                mdb.models[modelName].Equation(name='G-3-c12', terms=((1.0, 'c6', 3), (-1.0, 'c2', 3),(1.0, 'RP6', 3)))
                                mdb.models[modelName].Equation(name='G-3-c23', terms=((1.0, 'c2', 3), (-1.0, 'c3', 3),(-1.0, 'RP2', 3)))
                                mdb.models[modelName].Equation(name='G-3-c34', terms=((1.0, 'c3', 3), (-1.0, 'c4', 3),(1.0, 'RP5', 3)))
                                mdb.models[modelName].Equation(name='G-3-c45', terms=((1.0, 'c4', 3), (-1.0, 'c8', 3),(-1.0, 'RP6', 3)))
                                mdb.models[modelName].Equation(name='G-3-c56', terms=((1.0, 'c8', 3), (-1.0, 'c5', 3),(1.0, 'RP2', 3)))
                                mdb.models[modelName].Equation(name='G-3-c67', terms=((1.0, 'c5', 3), (-1.0, 'c1', 3),(1.0, 'RP6', 3)))
                                mdb.models[modelName].Equation(name='G-3-c78', terms=((1.0, 'c1', 3), (-1.0, 'c7', 3),(-1.0, 'RP2', 3),(-1.0, 'RP5', 3),(-1.0, 'RP6', 3)))
                 

                                for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                                    mdb.models[modelName].Equation(name='G-1-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 1), (-1.0, 'btedge%s'%k, 1),(-1.0, 'RP3', 1)))
                                    mdb.models[modelName].Equation(name='G-1-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 1), (-1.0, 'bbedge%s'%j, 1),(-1.0, 'RP4', 1)))
                                    mdb.models[modelName].Equation(name='G-1-bbedge-fbedge%s'%j, terms=((1.0, 'bbedge%s'%j, 1), (-1.0, 'fbedge%s'%l, 1),(1.0, 'RP3', 1)))
                                for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                                    mdb.models[modelName].Equation(name='G-2-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 2), (-1.0, 'btedge%s'%k, 2),(-1.0, 'RP4', 2)))
                                    mdb.models[modelName].Equation(name='G-2-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 2), (-1.0, 'bbedge%s'%j, 2),(-1.0, 'RP1', 2)))
                                    mdb.models[modelName].Equation(name='G-2-bbedge-fbedge%s'%j, terms=((1.0, 'bbedge%s'%j, 2), (-1.0, 'fbedge%s'%l, 2),(1.0, 'RP4', 2)))
                                for i,k,j,l in zip(ftedge,btedge,bbedge,fbedge):
                                    mdb.models[modelName].Equation(name='G-3-ftedge-btedge%s'%i, terms=((1.0, 'ftedge%s'%i, 3), (-1.0, 'btedge%s'%k, 3),(-1.0, 'RP5', 3)))
                                    mdb.models[modelName].Equation(name='G-3-btedge-bbedge%s'%k, terms=((1.0, 'btedge%s'%k, 3), (-1.0, 'bbedge%s'%j, 3),(-1.0, 'RP6', 3)))
                                    mdb.models[modelName].Equation(name='G-3-bbedge-fbedge%s'%j, terms=((1.0, 'bbedge%s'%j, 3), (-1.0, 'fbedge%s'%l, 3),(1.0, 'RP5', 3)))


                                for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                                    mdb.models[modelName].Equation(name='G-1-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 1), (-1.0, 'bledge%s'%k, 1),(-1.0, 'RP3', 1)))
                                    mdb.models[modelName].Equation(name='G-1-bledge-bredge%s'%k, terms=((1.0, 'bledge%s'%k, 1), (-1.0, 'bredge%s'%j, 1),(-1.0, 'RP5', 1)))
                                    mdb.models[modelName].Equation(name='G-1-bredge-fredge%s'%j, terms=((1.0, 'bredge%s'%j, 1), (-1.0, 'fredge%s'%l, 1),(1.0, 'RP3', 1)))
                                for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                                    mdb.models[modelName].Equation(name='G-2-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 2), (-1.0, 'bledge%s'%k, 2),(-1.0, 'RP4', 2)))
                                    mdb.models[modelName].Equation(name='G-2-bledge-bredge%s'%k, terms=((1.0, 'bledge%s'%k, 2), (-1.0, 'bredge%s'%j, 2),(-1.0, 'RP6', 2)))
                                    mdb.models[modelName].Equation(name='G-2-bredge-fredge%s'%j, terms=((1.0, 'bredge%s'%j, 2), (-1.0, 'fredge%s'%l, 2),(1.0, 'RP4', 2)))
                                for i,k,j,l in zip(fledge,bledge,bredge,fredge):
                                    mdb.models[modelName].Equation(name='G-3-fledge-bledge%s'%i, terms=((1.0, 'fledge%s'%i, 3), (-1.0, 'bledge%s'%k, 3),(-1.0, 'RP5', 3)))
                                    mdb.models[modelName].Equation(name='G-3-bledge-bredge%s'%k, terms=((1.0, 'bledge%s'%k, 3), (-1.0, 'bredge%s'%j, 3),(-1.0, 'RP2', 3)))
                                    mdb.models[modelName].Equation(name='G-3-bredge-fredge%s'%j, terms=((1.0, 'bredge%s'%j, 3), (-1.0, 'fredge%s'%l, 3),(1.0, 'RP5', 3)))
                                    

                                for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                                    mdb.models[modelName].Equation(name='G-1-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 1), (-1.0, 'lbedge%s'%k, 1),(-1.0, 'RP4', 1)))
                                    mdb.models[modelName].Equation(name='G-1-lbtedge-rbedge%s'%k, terms=((1.0, 'lbedge%s'%k, 1), (-1.0, 'rbedge%s'%j, 1),(-1.0, 'RP5', 1)))
                                    mdb.models[modelName].Equation(name='G-1-rbedge-rtbedge%s'%j, terms=((1.0, 'rbedge%s'%j, 1), (-1.0, 'rtedge%s'%l, 1),(1.0, 'RP4', 1)))                                    
                                for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                                    mdb.models[modelName].Equation(name='G-2-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 2), (-1.0, 'lbedge%s'%k, 2),(-1.0, 'RP1', 2)))
                                    mdb.models[modelName].Equation(name='G-2-lbtedge-rbedge%s'%k, terms=((1.0, 'lbedge%s'%k, 2), (-1.0, 'rbedge%s'%j, 2),(-1.0, 'RP6', 2)))
                                    mdb.models[modelName].Equation(name='G-2-rbedge-rtbedge%s'%j, terms=((1.0, 'rbedge%s'%j, 2), (-1.0, 'rtedge%s'%l, 2),(1.0, 'RP1', 2)))
                                for i,k,j,l in zip(ltedge,lbedge,rbedge,rtedge):
                                    mdb.models[modelName].Equation(name='G-3-ltedge-lbedge%s'%i, terms=((1.0, 'ltedge%s'%i, 3), (-1.0, 'lbedge%s'%k, 3),(-1.0, 'RP6', 3)))
                                    mdb.models[modelName].Equation(name='G-3-lbtedge-rbedge%s'%k, terms=((1.0, 'lbedge%s'%k, 3), (-1.0, 'rbedge%s'%j, 3),(-1.0, 'RP2', 3)))
                                    mdb.models[modelName].Equation(name='G-3-rbedge-rtbedge%s'%j, terms=((1.0, 'rbedge%s'%j, 3), (-1.0, 'rtedge%s'%l, 3),(1.0, 'RP6', 3)))

                        ## Shear modulus G12 ##
                        if G12==True and onlyPBC == False:
                                for i in mdb.models[modelName].loads.keys():
                                        del mdb.models[modelName].loads[i]
                                for i in mdb.models[modelName].boundaryConditions.keys():
                                        del mdb.models[modelName].boundaryConditions[i]

                                region = a.sets['RP4']
                                mdb.models[modelName].DisplacementBC(name='G12-1', createStepName='Step-1', 
                                    region=region, u1=Dispx, u2=Dispy, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)


                                region = a.sets['RP5']
                                mdb.models[modelName].DisplacementBC(name='G12-2', createStepName='Step-1', 
                                    region=region, u1=0, u2=0, u3=0, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)

                                region = a.sets['RP6']
                                mdb.models[modelName].DisplacementBC(name='G12-3', createStepName='Step-1', 
                                    region=region, u1=0, u2=0, u3=0, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)

                                regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                        del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                    createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                    sectionPoints=DEFAULT, rebar=EXCLUDE)

                                import os, glob

                                if mdb.jobs.has_key('job-G12'):
                                        del mdb.jobs['job-G12']
                                mdb.Job(name='job-G12', model= modelName, description='', type=ANALYSIS, 
                                    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                    scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                mdb.jobs['job-G12'].submit(consistencyChecking=OFF)

                                mdb.jobs['job-G12'].waitForCompletion()


                                o3 = session.openOdb(name='%s' % (path+'\job-G12.odb'))


                                odb = session.odbs['%s' % (path+'\job-G12.odb')]

                                session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                odbName=session.viewports[session.currentViewportName].odbDisplay.name


                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                    NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP4', ))

                                forceG12 = 0
                                for i in session.xyDataObjects.keys():
                                    forceG12=forceG12+(session.xyDataObjects[i][0][1])

                                stressG12 = abs(forceG12/(L*W))

                                G12 = stressG12/((Dispx/H)+(Dispy/L))

                        ## Shear modulus G13 ##                                
                        if G12==False or onlyPBC == True:
                                G12='N/A'


                        if G13==True and onlyPBC == False:
                                for i in mdb.models[modelName].loads.keys():
                                        del mdb.models[modelName].loads[i]
                                for i in mdb.models[modelName].boundaryConditions.keys():
                                        del mdb.models[modelName].boundaryConditions[i]
         

                                region = a.sets['RP5']
                                mdb.models[modelName].DisplacementBC(name='G13-1', createStepName='Step-1', 
                                    region=region, u1=Dispx, u2=UNSET, u3=Dispz, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)

                                region = a.sets['RP4']
                                mdb.models[modelName].DisplacementBC(name='G13-2', createStepName='Step-1', 
                                    region=region, u1=0, u2=0, u3=0, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)

                                region = a.sets['RP6']
                                mdb.models[modelName].DisplacementBC(name='G13-3', createStepName='Step-1', 
                                    region=region, u1=0, u2=0, u3=0, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)


                                regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                        del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                    createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                    sectionPoints=DEFAULT, rebar=EXCLUDE)

                                import os, glob

                                if mdb.jobs.has_key('job-G13'):
                                        del mdb.jobs['job-G13']
                                mdb.Job(name='job-G13', model= modelName, description='', type=ANALYSIS, 
                                    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                    scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                mdb.jobs['job-G13'].submit(consistencyChecking=OFF)

                                mdb.jobs['job-G13'].waitForCompletion()


                                o3 = session.openOdb(name='%s' % (path+'\job-G13.odb'))


                                odb = session.odbs['%s' % (path+'\job-G13.odb')]

                                session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                odbName=session.viewports[session.currentViewportName].odbDisplay.name



                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                    NODAL, ((COMPONENT, 'RF1'), )), ), nodeSets=('RP5', ))

                                forceG13 = 0
                                for i in session.xyDataObjects.keys():
                                    forceG13=forceG13+(session.xyDataObjects[i][0][1])

                                stressG13 = abs(forceG13/(H*L))


                                G13 = stressG13/((Dispx/W)+(Dispz/L))
                                
                        if G13==False or onlyPBC == True:
                                G13='N/A'

                        ## Shear modulus G23 ##
                        if G23==True and onlyPBC == False:
                                for i in mdb.models[modelName].loads.keys():
                                        del mdb.models[modelName].loads[i]
                                for i in mdb.models[modelName].boundaryConditions.keys():
                                        del mdb.models[modelName].boundaryConditions[i]
         

                                region = a.sets['RP6']
                                mdb.models[modelName].DisplacementBC(name='G23-1', createStepName='Step-1', 
                                    region=region, u1=UNSET, u2=Dispy, u3=Dispz, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)

                                region = a.sets['RP4']
                                mdb.models[modelName].DisplacementBC(name='G23-2', createStepName='Step-1', 
                                    region=region, u1=0, u2=0, u3=0, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)

                                region = a.sets['RP5']
                                mdb.models[modelName].DisplacementBC(name='G23-3', createStepName='Step-1', 
                                    region=region, u1=0, u2=0, u3=0, ur1=UNSET, ur2=UNSET, ur3=UNSET, 
                                    amplitude=UNSET, fixed=OFF, distributionType=UNIFORM, fieldName='', 
                                    localCsys=None)


                                regionDef=mdb.models[modelName].rootAssembly.sets['c1']
                                if mdb.models[modelName].historyOutputRequests.has_key('H-Output-2'):
                                        del mdb.models[modelName].historyOutputRequests['H-Output-2']
                                mdb.models[modelName].HistoryOutputRequest(name='H-Output-2', 
                                    createStepName='Step-1', variables=('RT', ), region=regionDef, 
                                    sectionPoints=DEFAULT, rebar=EXCLUDE)

                                import os, glob


                                if mdb.jobs.has_key('job-G23'):
                                        del mdb.jobs['job-G23']
                                mdb.Job(name='job-G23', model= modelName, description='', type=ANALYSIS, 
                                    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                    scratch='', multiprocessingMode=DEFAULT, numCpus=CPUs, numDomains=CPUs, numGPUs=0)
                                mdb.jobs['job-G23'].submit(consistencyChecking=OFF)

                                mdb.jobs['job-G23'].waitForCompletion()

                                o3 = session.openOdb(name='%s' % (path+'\job-G23.odb'))

                                odb = session.odbs['%s' % (path+'\job-G23.odb')]

                                session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                odbName=session.viewports[session.currentViewportName].odbDisplay.name

                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
                                    NODAL, ((COMPONENT, 'RF2'), )), ), nodeSets=('RP6', ))

                                forceG23 = 0
                                for i in session.xyDataObjects.keys():
                                    forceG23=forceG23+(session.xyDataObjects[i][0][1])

                                stressG23 = abs(forceG23/(L*H))


                                G23 = stressG23/((Dispy/W)+(Dispz/H))

                        if G23==False or onlyPBC == True:
                                G23='N/A'


#################################################


                        if CTE==True and onlyPBC == False:

                                for i in mdb.models[modelName].loads.keys():
                                        del mdb.models[modelName].loads[i]
                                for i in mdb.models[modelName].boundaryConditions.keys():
                                        del mdb.models[modelName].boundaryConditions[i]


                                nodeNo = []
                                for i in mdb.models[modelName].rootAssembly.allInstances[instanceName].nodes:
                                    nodeNo.append(int(i.label))

                                a.SetFromNodeLabels(name='CTE_part', nodeLabels=((instanceName,nodeNo),))

                                region = a.sets['CTE_part']
#                                region = a.instances[instanceName].sets['CTE_part']
                                mdb.models[modelName].Temperature(name='Predefined Field-1', 
                                    createStepName='Initial', region=region, distributionType=UNIFORM, 
                                    crossSectionDistribution=CONSTANT_THROUGH_THICKNESS, magnitudes=(intemp, ))
#                                        session.viewports['Viewport: 1'].assemblyDisplay.setValues(step='Step-1')
                                mdb.models[modelName].predefinedFields['Predefined Field-1'].setValuesInStep(
                                    stepName='Step-1', magnitudes=(fntemp, ))


                                if mdb.jobs.has_key('job-CTE'):
                                        del mdb.jobs['job-CTE']
                                mdb.Job(name='job-CTE', model=modelName, description='', type=ANALYSIS, 
                                    atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
                                    memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
                                    explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
                                    modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
                                    scratch='', multiprocessingMode=DEFAULT,  numCpus=CPUs, numDomains=CPUs, numGPUs=0)

                                mdb.jobs['job-CTE'].submit(consistencyChecking=OFF)
                                mdb.jobs['job-CTE'].waitForCompletion()

                                #Extract Data Needed to Find CTE

                                o3 = session.openOdb(name='%s' % (path+'\job-CTE.odb'))

                                odb = session.odbs['%s' % (path+'\job-CTE.odb')]


                                session.viewports['Viewport: 1'].setValues(displayedObject=o3)
                                odbName=session.viewports[session.currentViewportName].odbDisplay.name

                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]


                                session.viewports['Viewport: 1'].setValues(displayedObject=odb)
                                odbName=session.viewports[session.currentViewportName].odbDisplay.name
                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U1'), )), ), nodeSets=('C1','C2', ))
                                
                                C1U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][0]
                                
                                C2U1new = session.xyDataObjects['U:U1 PI: %s N: %s' % (upperName,c2[0])][0][1] + coc2[(c2[0])][0]


                                DistX = abs(C1U1new - C2U1new)
                                DeltaX = (L-DistX)
                                CTE_X = DeltaX/(L*(fntemp-intemp))


                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]
                                    

                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U2'), )), ), nodeSets=('C1','C5', ))
                                
                                C1U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][1]
                                
                                C5U2new = session.xyDataObjects['U:U2 PI: %s N: %s' % (upperName,c5[0])][0][1] + coc5[(c5[0])][1]



                                DistY = abs(C1U2new - C5U2new)
                                DeltaY = (H-DistY)
                                CTE_Y = DeltaY/(H*(fntemp-intemp))



                                for i in session.xyDataObjects.keys():
                                    del session.xyDataObjects['%s' % (i)]


                                session.odbData[odbName].setValues(activeFrames=(('Step-1', (1, )), ))
                                session.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('U', 
                                    NODAL, ((COMPONENT, 'U3'), )), ), nodeSets=('C1','C4', ))
                                
                                C1U3new = session.xyDataObjects['U:U3 PI: %s N: %s' % (upperName,c1[0])][0][1] + coc1[(c1[0])][2]
                                
                                C4U3new = session.xyDataObjects['U:U3 PI: %s N: %s' % (upperName,c4[0])][0][1] + coc4[(c4[0])][2]


                                DistZ = abs(C1U3new - C4U3new)
                                DeltaZ = (W-DistZ)
                                CTE_Z = DeltaZ/(W*(fntemp-intemp))


                        if CTE==False or onlyPBC == True:
                                CTE_X='N/A'
                                CTE_Y='N/A'
                                CTE_Z='N/A'

                        
                        density = 0
                        if mass != None:
                                density = mass/(L*W*H)
                        
                        print ('----------------------------------------------------')
                        print ('----------------------------------------------------')
                        print ('The homogenised elastic properties:')
                        print ('E11=%s Stress units' % (E11))
                        print ('V12=%s ratio' % (V12))
                        print ('V13=%s ratio' % (V13))
                        print ('E22=%s Stress units' % (E22))
                        print ('V21=%s ratio' % (V21))
                        print ('V23=%s ratio' % (V23))
                        print ('E33=%s Stress units' % (E33))
                        print ('V31=%s ratio' % (V31))
                        print ('V32=%s ratio' % (V32))
                        print ('G12=%s Stress units' % (G12))
                        print ('G13=%s Stress units' % (G13))
                        print ('G23=%s Stress units' % (G23))
                        print ('CTE X=%s N/A' % (CTE_X))
                        print ('CTE Y=%s N/A' % (CTE_Y))
                        print ('CTE Z=%s N/A' % (CTE_Z))
                        print ('----------------------------------------------------')
                        print ('Total mass=%s Mass units' % (mass))
                        print ('Homogenised density=%s Density units' % (density))
                        print ('----------------------------------------------------')
                        print ('Processing duration %s seconds' % (time.time()-start))
                        print ('----------------------------------------------------')
                        
                        filename = ('%s_elastic_properties.txt' % part)
                        print ('The homogenised elastic properties are saved in ABAQUS Work Directory under %s' % filename)
                        f = open(filename,'w')
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('Property','Value','Unit'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('E11',E11,'Stress units'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('V12',V12,'ratio'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('V13',V13,'ratio'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('E22',E22,'Stress units'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('V21',V21,'ratio'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('V23',V23,'ratio'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('E33',E33,'Stress units'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('V31',V31,'ratio'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('V32',V32,'ratio'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('G12',G12,'Stress units'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('G13',G13,'Stress units'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('G23',G23,'Stress units'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('CTE X',CTE_X,'N/A'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('CTE Y',CTE_Y,'N/A'))
                        f.write('{0:^10}{1:^20}{2:^20}\n'.format('CTE Z',CTE_Z,'N/A'))


                        f.write ('Total mass=%s Mass units \n' % (mass))
                        f.write ('Homogenised density=%s Density units \n' % (density))
                   
                        f.write ('processing duration %s Seconds' % (time.time()-start))

                        f.close()

                        print ('Citation: Omairey S, Dunning P, Sriramula S (2018) Development of an ABAQUS plugin tool for periodic RVE homogenisation.')
                        print ('Engineering with Computers. https://doi.org/10.1007/s00366-018-0616-4')


                        filename = ('%s_elastic_properties(easycopy).txt' % part)
                        f = open(filename,'w')
                        f.write('{0:^10}\n'.format(E11))
                        f.write('{0:^10}\n'.format(E22))
                        f.write('{0:^10}\n'.format(E33))
                        f.write('{0:^10}\n'.format(G12))
                        f.write('{0:^10}\n'.format(G13))
                        f.write('{0:^10}\n'.format(G23))
                        f.write('{0:^10}\n'.format(V12))
                        f.write('{0:^10}\n'.format(V13))
                        f.write('{0:^10}\n'.format(V21))
                        f.write('{0:^10}\n'.format(V23))
                        f.write('{0:^10}\n'.format(V31))
                        f.write('{0:^10}\n'.format(V32))
                        f.write('{0:^10}\n'.format(CTE_X))
                        f.write('{0:^10}\n'.format(CTE_Y))
                        f.write('{0:^10}\n'.format(CTE_Z))

                        f.write ('{0:^10}\n' .format(mass))
                        f.write ('{0:^10}\n' .format(density))                   
                        f.write ('{0:^10}\n' .format((time.time()-start)))

                        f.close()

                        print ('----------------------------------------------------')
                        if onlyPBC == True:
                                print ('EasyPBC created Period Boundary Conditions only. For further investigation, used relevant Reference Points to apply loads/displacements based on your needs. Details on the use of Reference Points are illustrated in Table 1 of the referred paper.')

                        for i in session.xyDataObjects.keys():
                            del session.xyDataObjects['%s' % (i)]
                        print ('---------------------------------------')
                        print ('--------- End of EasyPBC (3D) ---------')
                        print ('---------------------------------------')

                        if len(session.odbData.keys()) >= 1:                              
                                odb.close(odb, write=TRUE)
                                a = mdb.models[modelName].rootAssembly
                                session.viewports['Viewport: 1'].setValues(displayedObject=a)

                if error==True:
                        print ('Error(s) found during sets creation, please check the error No.(s) above with EasyPBC user guide.')
                        
                        a.SetFromNodeLabels(name='Error set', nodeLabels=((instanceName,errorset),))




if __name__ == '__main__':
    launch_gui()
