import pkgutil 

__path__ = pkgutil.extend_path(__path__, "modules")
for importer, modname, ispkg in pkgutil.walk_packages(path=__path__, prefix=__name__+'.'):
    if modname.endswith("pb2"):
        try:
            exec("from %s import *" % modname)
        except:
            pass

