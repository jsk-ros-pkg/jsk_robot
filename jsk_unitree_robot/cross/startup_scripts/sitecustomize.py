# install the apport exception handler if available
import site
import sys
import os.path as osp

site.PREFIXES += [
    '/opt/jsk/System/Python/',
    '/opt/jsk/System/ros1_dependencies/',
    '/opt/jsk/System/ros1_inst/',
]

paths = []
for path in site.getsitepackages():
    site_package = osp.join(osp.dirname(path), 'site-packages')
    if site_package != path and osp.exists(site_package):
        paths.append(site_package)
    else:
        paths.append(path)

sys.path.extend(paths)
try:
    import apport_python_hook
except ImportError:
    pass
else:
    apport_python_hook.install()
