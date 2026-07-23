#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

try:
    import vtk
except ImportError as exc:  # pragma: no cover - documentation helper only
    raise SystemExit("python3-vtk is required to render tool screenshots") from exc

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - used outside a sourced ROS env
    get_package_share_directory = None


PACKAGE_NAME = "dynamixel_detachable_hand"
TOOLS = ("gripper", "needle_holder")
VIEWS = {
    "front": {
        "direction": np.array([0.0, -1.0, 0.0]),
        "up": np.array([0.0, 0.0, 1.0]),
    },
    "side": {
        "direction": np.array([1.0, 0.0, 0.0]),
        "up": np.array([0.0, 0.0, 1.0]),
    },
}


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def package_share_path(package_name: str) -> Path:
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory(package_name))
        except Exception:
            pass
    root = package_root()
    if package_name == PACKAGE_NAME:
        return root
    sibling = root.parent / package_name
    if sibling.exists():
        return sibling
    raise FileNotFoundError(f"could not resolve package://{package_name}")


def resolve_mesh_uri(uri: str) -> Path:
    if uri.startswith("package://"):
        package_and_path = uri[len("package://") :]
        package_name, _, relative_path = package_and_path.partition("/")
        path = package_share_path(package_name) / relative_path
    else:
        path = Path(uri)
    if path.suffix.lower() == ".dae":
        stl_path = path.with_suffix(".stl")
        if stl_path.exists():
            return stl_path
    return path


def parse_triplet(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    values = [float(part) for part in text.split()]
    if len(values) != 3:
        raise ValueError(f"expected three values, got {text!r}")
    return np.array(values, dtype=float)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def transform_from_origin(origin: ET.Element | None) -> np.ndarray:
    matrix = np.eye(4)
    if origin is None:
        return matrix
    xyz = parse_triplet(origin.get("xyz"), (0.0, 0.0, 0.0))
    rpy = parse_triplet(origin.get("rpy"), (0.0, 0.0, 0.0))
    matrix[:3, :3] = rpy_matrix(rpy)
    matrix[:3, 3] = xyz
    return matrix


def scale_matrix(scale: str | None) -> np.ndarray:
    values = parse_triplet(scale, (1.0, 1.0, 1.0))
    matrix = np.eye(4)
    matrix[0, 0] = values[0]
    matrix[1, 1] = values[1]
    matrix[2, 2] = values[2]
    return matrix


def vtk_transform(matrix: np.ndarray) -> vtk.vtkTransform:
    vtk_matrix = vtk.vtkMatrix4x4()
    for row in range(4):
        for col in range(4):
            vtk_matrix.SetElement(row, col, float(matrix[row, col]))
    transform = vtk.vtkTransform()
    transform.SetMatrix(vtk_matrix)
    return transform


def child_link(joint: ET.Element) -> str:
    child = joint.find("child")
    if child is None or not child.get("link"):
        raise ValueError(f"joint {joint.get('name', '<unnamed>')} has no child link")
    return child.get("link", "")


def parent_link(joint: ET.Element) -> str:
    parent = joint.find("parent")
    if parent is None or not parent.get("link"):
        raise ValueError(f"joint {joint.get('name', '<unnamed>')} has no parent link")
    return parent.get("link", "")


def link_transforms(root: ET.Element, root_link: str) -> dict[str, np.ndarray]:
    children: dict[str, list[ET.Element]] = {}
    for joint in root.findall("joint"):
        child = child_link(joint)
        if child.endswith("_detach_link_0"):
            continue
        children.setdefault(parent_link(joint), []).append(joint)

    transforms = {root_link: np.eye(4)}
    queue = [root_link]
    while queue:
        parent = queue.pop(0)
        parent_transform = transforms[parent]
        for joint in children.get(parent, []):
            child = child_link(joint)
            transforms[child] = parent_transform @ transform_from_origin(joint.find("origin"))
            queue.append(child)
    return transforms


def link_color(link_name: str, tool: str) -> tuple[float, float, float]:
    if tool == "gripper" and ("finger" in link_name or "tip" in link_name):
        return (0.12, 0.32, 0.72)
    if "xl_xc" in link_name or "servo" in link_name:
        return (0.18, 0.19, 0.22)
    return (0.86, 0.88, 0.90)


def make_actor(mesh_path: Path, transform: np.ndarray, color: tuple[float, float, float]):
    if not mesh_path.exists():
        raise FileNotFoundError(mesh_path)
    if mesh_path.suffix.lower() != ".stl":
        return None
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(mesh_path))
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.SetUserTransform(vtk_transform(transform))
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetInterpolationToPhong()
    return actor


def actors_from_urdf(urdf_path: Path, tool: str, side: str) -> list[vtk.vtkActor]:
    root = ET.parse(urdf_path).getroot()
    transforms = link_transforms(root, f"{side}_base_link")
    actors = []
    for link in root.findall("link"):
        link_name = link.get("name", "")
        if link_name not in transforms or link_name.endswith("_detach_link_0"):
            continue
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh") if geometry is not None else None
            if mesh is None or not mesh.get("filename"):
                continue
            mesh_path = resolve_mesh_uri(mesh.get("filename", ""))
            transform = (
                transforms[link_name]
                @ transform_from_origin(visual.find("origin"))
                @ scale_matrix(mesh.get("scale"))
            )
            actor = make_actor(mesh_path, transform, link_color(link_name, tool))
            if actor is not None:
                actors.append(actor)
    return actors


def bounds_corners(bounds: tuple[float, float, float, float, float, float]) -> np.ndarray:
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    return np.array(
        [
            [x, y, z]
            for x in (xmin, xmax)
            for y in (ymin, ymax)
            for z in (zmin, zmax)
        ],
        dtype=float,
    )


def configure_camera(renderer, view_name: str, image_size: tuple[int, int]) -> None:
    bounds = renderer.ComputeVisiblePropBounds()
    corners = bounds_corners(bounds)
    center = corners.mean(axis=0)
    view = VIEWS[view_name]
    direction = view["direction"] / np.linalg.norm(view["direction"])
    up = view["up"] / np.linalg.norm(view["up"])
    right = np.cross(direction, up)
    right /= np.linalg.norm(right)
    width = max(1e-6, np.ptp(corners @ right))
    height = max(1e-6, np.ptp(corners @ up))
    depth = max(1e-6, np.ptp(corners @ direction))
    aspect = image_size[0] / image_size[1]

    camera = renderer.GetActiveCamera()
    camera.SetParallelProjection(True)
    camera.SetParallelScale(max(height * 0.5, width * 0.5 / aspect) * 1.20)
    camera.SetFocalPoint(*center)
    camera.SetPosition(*(center + direction * max(depth * 4.0, 0.5)))
    camera.SetViewUp(*up)
    renderer.ResetCameraClippingRange()


def render_view(
    *,
    tool: str,
    side: str,
    view_name: str,
    urdf_path: Path,
    output_path: Path,
    image_size: tuple[int, int],
) -> None:
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.96, 0.97, 0.98)
    for actor in actors_from_urdf(urdf_path, tool, side):
        renderer.AddActor(actor)

    light = vtk.vtkLight()
    light.SetLightTypeToSceneLight()
    light.SetPosition(0.3, -0.4, 0.6)
    light.SetFocalPoint(0.0, 0.0, 0.0)
    light.SetIntensity(0.85)
    renderer.AddLight(light)

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(True)
    window.SetSize(*image_size)
    window.AddRenderer(renderer)

    configure_camera(renderer, view_name, image_size)
    window.Render()

    image = vtk.vtkWindowToImageFilter()
    image.SetInput(window)
    image.Update()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(output_path))
    writer.SetInputConnection(image.GetOutputPort())
    writer.Write()


def render_tool(tool: str, side: str, output_dir: Path, image_size: tuple[int, int]) -> None:
    urdf_path = package_root() / "urdf" / f"{side}_{tool}.urdf"
    for view_name in VIEWS:
        render_view(
            tool=tool,
            side=side,
            view_name=view_name,
            urdf_path=urdf_path,
            output_path=output_dir / f"sim_{tool}_{view_name}.png",
            image_size=image_size,
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", default="rhand", choices=("lhand", "rhand"))
    parser.add_argument("--tools", nargs="+", default=list(TOOLS), choices=TOOLS)
    parser.add_argument("--output-dir", type=Path, default=package_root() / "figs")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args(argv)

    for tool in args.tools:
        render_tool(tool, args.side, args.output_dir, (args.width, args.height))
    return 0


if __name__ == "__main__":
    sys.exit(main())
