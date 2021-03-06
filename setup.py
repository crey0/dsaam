from setuptools import setup, find_packages
import subprocess

def get_version():
    p = subprocess.run("git describe | grep -o -E \"v[0-9]+(\\.[0-9]+)+(-[0-9]+)?\" -", shell=True, check=True,
                       universal_newlines=True, stdout=subprocess.PIPE)
    v = p.stdout.rstrip()
    return v.replace("-", ".dev", 1).replace("v", "", 1)

version = get_version()
print("Package version: " + version)

setup(
    name='dsaam',

    version=version,
    
    description='The DSAAM python library',
    #long_description="",

    url='https://redmine.laas.fr/projects/dsaam',

    author='Christophe Reymann',
    author_email='christophe.reymann@laas.fr',
    
    license='BSD-3-Clause',

    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3.5',
    ],
    package_dir={'':'python'},
    packages=find_packages(where='python'),
    install_requires = [],
    
)
