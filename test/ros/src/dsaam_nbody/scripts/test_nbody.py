#!/usr/bin/env python3
import yaml
from dsaam.node import Time
import numpy as np
import os
import errno
import subprocess
import re
import sys

ROS_PKG_PATH = "ROS_PACKAGE_PATH"

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

def pkg_root_path(path):
    f = path.split("/")
    f.reverse()
    i = f.index("dsaam_nbody")
    f.reverse()
    return "/".join(f[:-1-i])

LAUNCH = '<launch> \n @BODY@ \n </launch>'

GROUP ='\
        <group ns="@NAMESPACE@">\n\
        <rosparam command="load" file="@YAML@" />\n\
        <node type="@EXECUTABLE@"\n\
              name="@NAME@"\n\
              pkg="@PKG@"\n\
              output="screen" respawn="false" required="true"\n\
              cwd="node"\n\
        />\
        </group>'

ROSPARAM = '<rosparam command="load" file="@YAML@" />'

def create_yaml(path, pdict):
    mkdir_p_file(path)
    with open(path, 'w') as f:
        yaml.dump(pdict, f, default_flow_style=False)

class LaunchFileCreator:

    def __init__(self):
        self.strings = []

    def node(self, name, namespace, exec_path, yaml_path, pkg_path):
        self.strings.append(
            GROUP.replace("@NAME@", name)\
            .replace("@NAMESPACE@", namespace)\
            .replace("@YAML@", yaml_path)\
            .replace("@EXECUTABLE@", exec_path)\
            .replace("@PKG@", pkg_path))
        return self
        
    def rosparam(self, yaml_path):
        self.strings.append(
            ROSPARAM.replace("@YAML@", yaml_path))
        return self

    def write(self, path):
        string = LAUNCH.replace("@BODY@", "\n".join(self.strings))
        mkdir_p_file(path)
        with open(path, 'w') as f:
            f.write(string)
        
def create_launch_file(path, autotest=True):
    # BEGIN PARAMS
    G = 1.5 * 1.80 * 1e-11
    C = 8 * 1e-3
    eps = 1e-3
    colors = ['red', 'green', 'blue', 'yellow']
    n = {
        'red': 1,
        'green': 1,
        'blue': 1,
        'yellow':1,
        'drawer':1,
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
    if not autotest:
        stop_time = Time(0)
    else:
        stop_time = Time(10)
    max_qsize = 10

    draw_size = (10, 10)
    draw_scale = 1

    effectors = {
        'red': ['red', 'yellow'],
        'green': ['red', 'green'],
        'blue': ['green', 'blue'],
        'yellow': ['red', 'green', 'blue'],
        'drawer': ['red', 'green', 'blue', 'yellow']}
        
    idx = {
        'red': ['red_{}'.format(i) for i in range(n['red'])],
        'green':['green_{}'.format(i) for i in range(n['green'])],
        'blue': ['blue_{}'.format(i) for i in range(n['blue'])],
        'yellow': ['yellow_{}'.format(i) for i in range(n['yellow'])],
        'drawer': ['drawer_{}'.format(i) for i in range(n['drawer'])],
        }

    m_types = [("position", "geometry_msgs.msg.PointStamped"),
               ("speed",    "geometry_msgs.msg.QuaternionStamped")]
    
    # END PARAMS

    runfile = "dsaam_nbody_node"
    runfile_py = "run_ros_node.sh"
    conf_path = path + "/../launch/"
    launch = LaunchFileCreator()
    global_params = { 
        'autotest':autotest,
        'stop_time':stop_time.sec
    }
    create_yaml(conf_path + "globals.yaml", global_params)
    launch.rosparam(conf_path + "globals.yaml")
    for c in colors:
        for num, i in enumerate(idx[c]):
            body_params = {
                'position': positions[c][num].tolist(),
                'speed': speeds[c][num].tolist(),
                'mass': masses[c][num].tolist(),
                'radius': radii[c][num].tolist(),
                'color': c
            }
            outflows = [{
                'from': i+"/"+i,
                'name': i + "/" + mt[0],
                'message_class': mt[1],
                'dt': dt[c].to_nanos(),
                'sinks': [j+"/"+j for color in idx for j in idx[color]
                          if c in effectors[color] and not j == i],
            } for mt in m_types ]
            inflows = [{
                'from': j+"/"+j,
                'name': j + "/" + mt[0],
                'message_class': mt[1],
                'dt': dt[color].to_nanos(),
                } for color in effectors[c] for j in idx[color] for mt in m_types
                       if not j == i ]
            node_params= {
                'name': i+"/"+i,
                'max_qsize': max_qsize,
                'start_time': start_time.to_nanos(),
                'dt': dt[c].to_nanos(),
                'outflows': outflows,
                'inflows': inflows,
                'body': body_params
            }
            yamlp = conf_path + "{}.yaml".format(i)
            create_yaml(yamlp, node_params)
            if c == 'blue':
                launch.node(i, i, runfile_py, yamlp, "dsaam_nbody")
            else:
                launch.node(i, i, runfile, yamlp, "dsaam_nbody")

    
    c = 'drawer'
    i = 'drawer_0'
    outflows = []
    inflows = [{
        'from': j+"/"+j,
        'name': j + "/" + mt[0],
        'message_class': mt[1],
        'dt': dt[color].to_nanos(),
    } for color in effectors[c] for j in idx[color] for mt in m_types] 
    node_params= {
        'name': i+"/"+i,
        'max_qsize': max_qsize,
        'start_time': start_time.to_nanos(),
        'dt': dt[c].to_nanos(),
        'outflows': outflows,
        'inflows': inflows,
        'size': draw_size,
        'scale': draw_scale,
    }
    yamlp = conf_path + "{}.yaml".format(i)
    create_yaml(yamlp, node_params)
    launch.node(i, i, runfile_py, yamlp, "dsaam_nbody")

    launch.write(conf_path + "test_nbody.launch")

    # Set ros package path to add the dsaam python root folder 
    rospkgp = ""
    if ROS_PKG_PATH in os.environ:
        rospkgp = os.environ[ROS_PKG_PATH]
    pkg_root = pkg_root_path(path)
    rospkgp =  pkg_root + ":" + rospkgp
    os.environ[ROS_PKG_PATH] = rospkgp
    return conf_path + "test_nbody.launch"

def test_ros_nbody(autotest=True):
    path = os.path.abspath(os.path.dirname(__file__))
    launch_path = create_launch_file(path, autotest)
    cmd = "roslaunch {}".format(launch_path)
    p = subprocess.run(cmd, shell=True, check=True, stderr=subprocess.PIPE, universal_newlines=True)
    if p.stderr is not None:
        rex = re.compile('exit code (\d+)')
        m = rex.search(p.stderr)
        code = 0
        if m is not None:
            code = int(m.group(1))
        print(p.stderr, file=sys.stderr)
        assert code == 0, "One of the processes died with non-zero exit code {}".format(code)

if __name__ == "__main__":
    test_ros_nbody(autotest=False)
