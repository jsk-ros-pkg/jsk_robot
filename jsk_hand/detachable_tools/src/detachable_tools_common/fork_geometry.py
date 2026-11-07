"""Geometric seating checks against the actual gripper collision STL meshes."""
from pathlib import Path
from urllib.parse import urlparse, unquote
import xml.etree.ElementTree as ET
import numpy as np
from .fork_simulation import transform


def link_pose(root, child, positions=None, ancestor=None):
    positions = positions or {}
    result = np.eye(4)
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    while child in joints and child != ancestor:
        j = joints[child]
        o = j.find('origin')
        local = np.eye(4) if o is None else transform(
            np.fromstring(o.get('xyz', '0 0 0'), sep=' '),
            np.fromstring(o.get('rpy', '0 0 0'), sep=' '))
        axis = j.find('axis')
        vector = np.array([1.,0,0]) if axis is None else np.fromstring(axis.get('xyz'), sep=' ')
        motion = np.eye(4)
        value = positions.get(j.get('name'), 0.)
        if j.get('type') in ('revolute', 'continuous'):
            from scipy.spatial.transform import Rotation
            motion[:3,:3] = Rotation.from_rotvec(vector*value).as_matrix()
        elif j.get('type') == 'prismatic':
            motion[:3,3] = vector*value
        result = local @ motion @ result
        child = j.find('parent').get('link')
    if ancestor is not None and child != ancestor:
        raise ValueError(f'No chain to {ancestor}')
    return result


def collision_triangles(tool_xml, tool_pose):
    root = ET.fromstring(tool_xml)
    dtype = np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
    result = {}
    for link in root.findall('link'):
        triangles = []
        for col in link.findall('collision'):
            mesh = col.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Seat validation requires STL collision geometry')
            uri = urlparse(mesh.get('filename'))
            if uri.scheme != 'file':
                raise ValueError('Resolve mesh URIs before validating the seat')
            data = Path(unquote(uri.path)).read_bytes()
            count = int.from_bytes(data[80:84], 'little')
            if len(data) != 84 + 50*count:
                raise ValueError('Expected binary STL')
            vertices = np.frombuffer(data,dtype,offset=84)['vertices'].astype(float)
            vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            origin = col.find('origin')
            local = np.eye(4) if origin is None else transform(
                np.fromstring(origin.get('xyz', '0 0 0'),sep=' '),
                np.fromstring(origin.get('rpy', '0 0 0'),sep=' '))
            pose = tool_pose @ link_pose(root, link.get('name')) @ local
            triangles.append(vertices @ pose[:3,:3].T + pose[:3,3])
        if triangles:
            result[link.get('name')] = np.concatenate(triangles)
    return result


def slab_vertices(triangles, bottom, top):
    def clip(poly, z, above):
        out = []
        for a,b in zip(poly, poly[1:]+poly[:1]):
            ina = a[2]>=z if above else a[2]<=z
            inb = b[2]>=z if above else b[2]<=z
            if ina:
                out.append(a)
            if ina != inb:
                out.append(a+(b-a)*(z-a[2])/(b[2]-a[2]))
        return out
    points = []
    active = triangles[(triangles[:,:,2].max(1)>=bottom)&(triangles[:,:,2].min(1)<=top)]
    for tri in active:
        poly = clip(list(tri), bottom, True)
        if poly:
            points.extend(clip(poly, top, False))
    return np.asarray(points).reshape(-1,3)


def validate_seating(tool_xml, config):
    """Check the fork slab along the straight insertion, plus housing support height.

    Meshes are clipped to the slab interior (1 um contact tolerance). For a U slot,
    moving toward its entrance can only increase clearance for these tool sections.
    This is a local geometric check, not robot-wide collision checking or physics.
    """
    pose = transform(config['fork_to_tool_xyz'], config['fork_to_tool_rpy'])
    meshes = collision_triangles(tool_xml, pose)
    half = config['fork_thickness']/2
    radius = config['tool_neck_radius'] + config['fork_radial_clearance']
    gap = float('inf')
    section_points = []
    for triangles in meshes.values():
        vertices = slab_vertices(triangles, -half+1e-6, half-1e-6)
        if not len(vertices):
            continue
        section_points.append(vertices)
        y = vertices[:,1] * config['entrance_sign']
        required = np.where(y<0, np.linalg.norm(vertices[:,:2],axis=1), np.abs(vertices[:,0]))
        gap = min(gap, float(np.min(radius-required)))
    if not section_points:
        raise RuntimeError('Tool does not enter the fork slab')
    if gap < -1e-6:
        raise RuntimeError(f'Tool intersects fork: {-gap*1000:.3f} mm penetration')
    # Gripper housing underside rests on the fork, while the fingers pass through.
    housing = meshes['rhand_base_link'].reshape(-1,3)
    support_gap = float(housing[:,2].min()-half)
    support_vertices = housing[(housing[:,2] <= half+2e-6) & (np.abs(housing[:,0]) > radius)]
    if abs(support_gap) > 1e-5 or not len(support_vertices):
        raise RuntimeError(f'Housing is not supported by fork top: gap {support_gap:.6f} m')
    section = np.concatenate(section_points)
    start_y = config['approach_offset'][1]*config['entrance_sign'] + section[:,1].min()
    if start_y <= config['fork_mouth_depth']:
        raise RuntimeError('Approach does not fully clear the shortened fork entrance')
    return {'local_mesh_check_passed': True, 'minimum_slot_clearance_m': gap,
            'housing_support_gap_m': support_gap,
            'tool_axis_to_seat_center_m': float(np.linalg.norm(pose[:2,3])),
            'fork_mouth_depth_m': config['fork_mouth_depth'],
            'scope': 'closed gripper vs fork slab; no robot-wide collision or contact dynamics'}
