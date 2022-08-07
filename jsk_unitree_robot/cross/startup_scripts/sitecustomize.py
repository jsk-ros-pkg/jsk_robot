# install the apport exception handler if available
import site
import sys
import os
import os.path as osp

site.PREFIXES += [
    '/opt/jsk/System/Python/',
    '/opt/jsk/System/ros1_dependencies/',
    '/opt/jsk/System/ros1_inst/',
]


def recursively_listup_paths(path):
    paths = []
    for sub_path in os.listdir(path):
        sub_full_path = osp.join(path, sub_path)
        if osp.isdir(sub_full_path) and (sub_path.endswith('.egg-info') or sub_path.endswith('.egg')):
            paths.append(sub_full_path)
    return paths


paths = []
for path in site.getsitepackages():
    if osp.exists(path):
        paths.append(path)
        paths.extend(recursively_listup_paths(path))
    site_package = osp.join(osp.dirname(path), 'site-packages')
    if site_package != path and osp.exists(site_package):
        paths.append(site_package)
        paths.extend(recursively_listup_paths(site_package))

sys.path.extend(paths)
try:
    import apport_python_hook
except ImportError:
    pass
else:
    apport_python_hook.install()
