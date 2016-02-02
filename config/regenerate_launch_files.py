#!/usr/bin/env python2
# (C) 2015  Jean Nassar
# Released under BSD
import glob
import os
import subprocess as sp

import rospkg
import tqdm


def get_launch_dir(package):
    return os.path.join(rospkg.RosPack().get_path(package), "launch")


def get_file_root(path):
    """
    >>> get_file_root("/tmp/test.txt")
    'test'

    """
    return os.path.split(path[:path.rindex(".")])[1]


def compile_xacro(inpath, outpath, stdout):
    sp.call("rosrun xacro xacro {inpath} --inorder -o {outpath}"
            .format(inpath=inpath, outpath=outpath).split(),
            stdout=stdout)


def main():
    launch_dir = get_launch_dir("spirit")
    os.chdir(launch_dir)
    with open(os.devnull, "w") as DEVNULL:
        for path in tqdm.tqdm(glob.glob("xacro/*.xacro"),
                              desc="Regenerating launch files"):
            root = get_file_root(path)
            compile_xacro(path, os.path.join("launchers", root), DEVNULL)


if __name__ == "__main__":
    main()
