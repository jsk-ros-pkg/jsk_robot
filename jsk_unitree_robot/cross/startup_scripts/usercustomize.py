import site
import sys
import os
import os.path as osp

site.PREFIXES += [
    '/opt/jsk/System/Python/',
    '/opt/jsk/System/ros1_dependencies/'
]

# https://www.programcreek.com/python/?code=Pylons%2Fhupper%2Fhupper-master%2Fsrc%2Fhupper%2Fcompat.py
def get_site_packages():
    try:
        paths = []
        for path in site.getsitepackages():
            if osp.exists(path):
                paths.append(path)
            site_package = osp.join(osp.dirname(path), 'site-packages')
            if site_package != path and osp.exists(site_package):
                paths.append(site_package)

        return paths

    # virtualenv does not ship with a getsitepackages impl so we fallback
    # to using distutils if we can
    # https://github.com/pypa/virtualenv/issues/355
    except Exception:
        try:
            paths = []
            for base_path in site.PREFIXES:
                for python_path in ['lib/python{}'.format(sys.version_info[0]),
                                    'lib/python{}.{}'.format(sys.version_info[0], sys.version_info[1])]:
                    site_package = osp.join(base_path, python_path, 'site-packages')
                    if osp.exists(site_package):
                        paths.append(site_package)
            return paths

        # just incase, don't fail here, it's not worth it
        except Exception as e:
            return []

sys.path.extend(get_site_packages())
