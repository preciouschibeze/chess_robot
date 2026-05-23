from __future__ import absolute_import

import argparse
import os
import sys
import xml.etree.ElementTree as ET


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, help="URDF file to inspect.")
    return parser


def main():
    args = build_parser().parse_args()
    urdf_dir = os.path.dirname(os.path.abspath(args.urdf))
    root = ET.parse(args.urdf).getroot()

    mesh_filenames = []
    for mesh_node in root.findall(".//mesh"):
        filename = mesh_node.attrib.get("filename")
        if filename:
            mesh_filenames.append(filename)

    missing = []
    for filename in mesh_filenames:
        resolved_path = resolve_mesh_path(filename, urdf_dir)
        exists = resolved_path is not None and os.path.exists(resolved_path)
        print("%s" % filename)
        print("  resolved: %s" % (resolved_path if resolved_path is not None else "unresolved"))
        print("  exists: %s" % ("yes" if exists else "no"))
        if not exists:
            missing.append(filename)

    print("Mesh files checked: %d" % len(mesh_filenames))
    print("Missing mesh files: %d" % len(missing))
    if missing:
        print("Missing list:")
        for filename in missing:
            print("  %s" % filename)


def resolve_mesh_path(filename, urdf_dir):
    if filename.startswith("package://"):
        return None
    if os.path.isabs(filename):
        return filename
    return os.path.normpath(os.path.join(urdf_dir, filename))


if __name__ == "__main__":
    main()
