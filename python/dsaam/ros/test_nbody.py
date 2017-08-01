import yaml
from ..node import Time
import numpy as np
import os
import errno
import subprocess

def mkdir_p_file(path):
    fpath = os.path.abspath(path)
    dpath = os.path.dirname(fpath)
    try:
        os.makedirs(dpath)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(dpath):
            pass
        else:
            raise exc
    return fpath

LAUNCH = '<launch> \n @BODY@ \n </launch>'

GROUP = '<group ns="@NAMESPACE@">\
        <rosparam command="load" file="@YAML@" />\
        <node type="@PYTHONFILE@"\
              name="@NAME@"\
              output="screen" respawn="false" required="true"\
              cwd="node"\
              launch-prefix="$(optenv PYTHON3_INTERPRETER)"/>\
        </group>'

ROSPARAM = '<rosparam command="load" file="@YAML@" />'

def create_yaml(path, pdict):
    mkdir_p_file(path)
    with open(path, 'w') as f:
        yaml.dump(pdict, f, default_flow_style=False)

class LaunchFileCreator:

    def __init__(self):
        self.strings = []

    def node(self, name, namespace, yaml_path, exec_path):
        self.strings.append(
            GROUP.replace("@NAME@", name)\
            .replace("@NAMESPACE@", namespace)\
            .replace("@YAML@", yaml_path)\
            .replace("@PYTHONFILE@", exec_path))
        return self
        
    def rosparam(self, yaml_path):
        self.strings.append(
            ROSPARAM.replace("@YAML@", yaml_path))
        return self

    def write(self, path):
        string = LAUNCH.replace("@BODY@", "\n".concatenate(self.strings))
        mkdir_p_file(path)
        with open(path, 'w') as f:
            f.write(string)
        
def create_launch_file(path, autotest=False):
    # BEGIN PARAMS
    G = 1.5 * 1.80 * 1e-11
    C = 8 * 1e-3
    eps = 1e-3
    colors = ['red', 'green', 'blue', 'yellow']
    n = {
        'red': 1,
        'green': 2,
        'blue': 3,
        'yellow':5,
    }
    positions = {c: np.random.random((n[c], 2)) for c in colors}
    positions['red'][0] = [0.5, 0.5]
    speeds = {
        'red': np.zeros((n['red'], 2)),
        'green':  0.1 * C * np.random.random((n['green'], 2)) - 0.05*C,
        'blue':  0.1 * C * np.random.random((n['blue'],2)),
        'yellow': 0.1 * C * np.random.random((n['yellow'],2)) - 0.05*C,
    }
    masses = {
        'red':    (1+np.random.random(n['red']))*10000,
        'green':  (1+np.random.random(n['green']))*1000,
        'blue':   (1+np.random.random(n['blue']))*100,
        'yellow': np.ones(n['yellow'])*100,
    }
    radii = {
        'red':    masses['red']*0.1,
        'green':  masses['green']*0.1,
        'blue':   masses['blue']*0.1,
        'yellow': masses['yellow']*0.1,
    }
    dt = {
        'red':    Time(0,int(1e8)),
        'green':  Time(0,int(1e8)),
        'blue':   Time(0,int(1e8)),
        'yellow': Time(0,int(1e8)),
        'drawer': Time(1, int(1e8)),
    }
     
    start_time = Time(0)
    max_qsize = 10

    draw_size = (10, 10)
    draw_scale = 1

    effectors = {
        'red': ['red', 'yellow'],
        'green': ['red', 'green'],
        'blue': ['green', 'blue'],
        'yellow': ['red', 'green', 'blue'],
        'drawer': ['red, green','blue', 'yellow']}
        
    idx = {
        'red': ['red-{}'.format(i) for i in range(n['red'])],
        'green':['green-{}'.format(i) for i in range(n['green'])],
        'blue': ['blue-{}'.format(i) for i in range(n['blue'])],
        'yellow':['yellow-{}'.format(i) for i in range(n['yellow'])]}
    # END PARAMS

    conf_path = path + "/conf/"
    launch = LaunchFileCreator()
    global_params = { 
        'autotest':autotest,
    }
    create_yaml(conf_path + "globals.yaml", global_params)
    launch.rosparam(conf_path + "globals.yaml")
    for c in colors:
        for num, i in enumerate(idx[c]):
            body_params = {
                'position': positions[c][num],
                'speed': speeds[c][num],
                'mass': masses[c][num],
                'radius': radii[c][num],
                'color': c
            }
            outflows = [{
                'name': i,
                'message_class': "geometry_msgs.msg.PointStamped",
                'dt': dt[c],
                'sinks': [j for color in colors for j in idx[color] if c in effectors[color]],
            }]
            inflows = [{
                'name': j,
                'message_class': "geometry_msgs.msg.PointStamped",
                'dt': dt[color],
                } for color in effectors[c] for j in idx[color]]
            node_params= {
                'max_qsize': max_qsize,
                'start_time': start_time,
                'dt': dt[c],
                'outflows': outflows,
                'inflows': inflows,
                'body': body_params
            }
            yamlp = conf_path + "{}.yaml".format(i)
            create_yaml(yamlp, node_params)
            launch.node(i, i, path + "/node.py", yamlp)
    launch.write(conf_path + "test_nbody.launch")
    return conf_path + "test_nbody.launch"

def test_ros_nbody(autotest=True):
    path = os.path.abspath(os.path.dirname(__file__))
    launch_path = create_launch_file(path, autotest)
    cmd = "roslaunch {}".format(launch_path)
    subprocess.run(cmd, shell=True, check=True)
if __name__ == "__main__":
    test_ros_nbody()
