try:
    from urllib.request import urlopen
except ImportError:
    from urllib2 import urlopen
import yaml


class DevelDistro:
    def __init__(self, name):
        url = urlopen('https://raw.github.com/ros/rosdistro/master/releases/{0}-devel.yaml'.format(name))
        distro = yaml.load(url.read())['repositories']
        self.repositories = {}
        for name, data in distro.iteritems():
            repo = DevelDistroRepo(name, data)
            self.repositories[name] = repo


class DevelDistroRepo:
    def __init__(self, name, data):
        self.name = name
        self.type = data['type']
        self.url = data['url']
        self.version = None
        if 'version' in data:
            self.version = data['version']

    def get_rosinstall(self):
        if self.version:
            return yaml.dump([{
                self.type: {
                    'local-name': self.name,
                    'uri': '{0}'.format(self.url),
                    'version': '{0}'.format(self.version)
                }}], default_style=False)
        else:
            return yaml.dump([{
                self.type: {
                    'local-name': self.name,
                    'uri': '{0}'.format(self.url)
                }}], default_style=False)
