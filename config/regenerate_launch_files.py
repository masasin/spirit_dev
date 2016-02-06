#!/usr/bin/env python2
# (C) 2015  Jean Nassar
# Released under BSD
import glob
from lxml import etree as et
import os
import subprocess as sp

import rospkg
import tqdm
import yaml


namespace = {"xacro": "{http://www.ros.org/wiki/xacro}"}
# et.register_namespace("xacro", "http://www.ros.org/wiki/xacro")


def get_ros_dir(package, dirname):
    return os.path.join(rospkg.RosPack().get_path(package), dirname)


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
    

def get_past_image_keys():
    with open("launch_params.yaml") as fin:
        launch_params = yaml.load(fin)

    keys = {}
    for eval_method in launch_params["past_image"]:
        if eval_method != "eval_method":
            keys[eval_method] = launch_params["past_image"][eval_method].keys()
    return keys


def load_xml(path):
    parser = et.XMLParser(remove_blank_text=True)
    return et.parse(path, parser)
    

def remove_old_elements(node):
    for element in node.findall("{}if".format(namespace["xacro"])):
        node.remove(element)


def add_new_keys(node, keys):
    for method_name, key_list in keys.items():
        element = et.Element("{}if".format(namespace["xacro"]),
                             attrib={"value": "${{method == '{}'}}"
                                     .format(method_name)})
        for key in key_list:
            et.SubElement(element, "param",
                          attrib={"name": key,
                                  "value": "${{method_ns['{}']}}".format(key)})
        node.append(element)


def add_message(tree):
    root = tree.getroot()
    root.addprevious(et.Comment("Generated automatically from launch config file."))


def update_past_image_generator(keys, path="xacro/past_images.launch.xacro"):
    tree = load_xml(path)
    for node in tree.findall("node[@name='past_image_selector']"):
        remove_old_elements(node)
        add_new_keys(node, keys)
    add_message(tree)
    tree.write(path, encoding="utf-8", xml_declaration=True, pretty_print=True)


def main():
    os.chdir(get_ros_dir("spirit", "config"))
    past_image_keys = get_past_image_keys()

    os.chdir(get_ros_dir("spirit", "launch"))
    update_past_image_generator(past_image_keys)

    with open(os.devnull, "w") as DEVNULL:
        for path in tqdm.tqdm(glob.glob("xacro/*.xacro"),
                              desc="Regenerating launch files",
                              unit=" files",
                              leave=True):
            root = get_file_root(path)
            compile_xacro(path, os.path.join("launchers", root), DEVNULL)


if __name__ == "__main__":
    main()
