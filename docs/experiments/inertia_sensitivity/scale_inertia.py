"""List link masses, or write a copy of the URDF with some inertia tensors scaled.

Mass and centre of mass are left alone; only ixx..izz of the named links move,
which is the parameter the feedforward's inertia term is linear in.
"""
import argparse
import xml.etree.ElementTree as ET

KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")

parser = argparse.ArgumentParser()
parser.add_argument("urdf")
parser.add_argument("--list", action="store_true")
parser.add_argument("--scale", type=float, default=1.0)
parser.add_argument("--links", default="")
parser.add_argument("--out")
args = parser.parse_args()

tree = ET.parse(args.urdf)
root = tree.getroot()
links = {link.get("name"): link for link in root.findall("link")}

if args.list:
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        if mass <= 1e-3:
            continue
        inertia = inertial.find("inertia")
        print(f"{name:28} {mass:9.4f} kg  " +
              " ".join(f"{k}={float(inertia.get(k)):.6g}" for k in ("ixx", "iyy", "izz")))
    raise SystemExit

wanted = [n for n in args.links.split(",") if n]
missing = [n for n in wanted if n not in links]
assert not missing, f"no such link: {missing}"
for name in wanted:
    inertia = links[name].find("inertial").find("inertia")
    for key in KEYS:
        if inertia.get(key) is not None:
            inertia.set(key, repr(args.scale*float(inertia.get(key))))
tree.write(args.out)
print(f"wrote {args.out}: scaled {len(wanted)} links by {args.scale}")
