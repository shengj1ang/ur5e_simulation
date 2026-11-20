"""List link masses, or write a copy of the URDF with model parameters perturbed.

Three parameters can be moved, alone or together, on a chosen set of links:

  --scale       the rotational inertia tensor, ixx..izz
  --mass-scale  the mass
  --com-offset  the centre of mass, i.e. the inertial origin's xyz, in metres

Links are chosen by name with --links, or with --subtree-of LINK, which selects
every link strictly beyond LINK and so matches the payload walk in
distal_inertia.h - the way the controller folds the coupler, gripper and camera
into its model of the tip.

What each parameter can reach. The feedforward's two terms are M*qdd_ref and
G(q; -e_axis), and qdd_ref comes from the Jacobian alone, so none of these move
it. --scale therefore moves the inertia term only, which is why sweeping it
alone gives a narrow range: it cannot touch the gravity term that the inertia
term nearly cancels. Mass reaches both terms, and a uniform mass error reaches
both nearly proportionally, so it scales the difference without threatening its
sign. It is the differential errors - one link against another, the payload
against the arm, and the centres of mass - that move the two terms by different
amounts and so move their difference.

--com-offset moves each selected link's centre of mass by this vector in that
link's own frame. For a cluster of links whose frames are not aligned, such as
the payload, the aggregate centre of mass does not move by that same vector;
the realized one is in the '# payload_mass_kg ... cog' line that
inertia_sensitivity prints for every perturbed model.
"""
import argparse
import xml.etree.ElementTree as ET

KEYS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")

parser = argparse.ArgumentParser()
parser.add_argument("urdf")
parser.add_argument("--list", action="store_true")
parser.add_argument("--scale", type=float, default=1.0,
                    help="rotational inertia tensor scale")
parser.add_argument("--mass-scale", type=float, default=1.0)
parser.add_argument("--com-offset", default="", metavar="DX,DY,DZ",
                    help="metres, in each selected link's own frame")
parser.add_argument("--links", default="")
parser.add_argument("--subtree-of", default="", metavar="LINK",
                    help="select every link strictly beyond LINK")
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

offset = [0.0, 0.0, 0.0]
if args.com_offset:
    parts = args.com_offset.split(",")
    assert len(parts) == 3, f"--com-offset wants three numbers, got {args.com_offset!r}"
    offset = [float(p) for p in parts]

children = {}
for joint in root.findall("joint"):
    children.setdefault(joint.find("parent").get("link"), []).append(
        joint.find("child").get("link"))


def subtree(name):
    """Every link strictly beyond name, in the order distal_inertia.h walks."""
    found = []
    for child in children.get(name, []):
        found.append(child)
        found.extend(subtree(child))
    return found


wanted = [n for n in args.links.split(",") if n]
if args.subtree_of:
    assert args.subtree_of in links, f"no such link: {args.subtree_of}"
    wanted += subtree(args.subtree_of)
missing = [n for n in wanted if n not in links]
assert not missing, f"no such link: {missing}"
perturbed = args.scale != 1.0 or args.mass_scale != 1.0 or any(offset)
assert wanted or not perturbed, "nothing selected: pass --links or --subtree-of"

touched = 0
selected_mass = 0.0
for name in dict.fromkeys(wanted):          # de-duplicated, order kept
    inertial = links[name].find("inertial")
    if inertial is None:
        continue
    touched += 1

    mass = inertial.find("mass")
    if mass is not None:
        value = args.mass_scale * float(mass.get("value"))
        mass.set("value", repr(value))
        selected_mass += value

    inertia = inertial.find("inertia")
    if inertia is not None:
        for key in KEYS:
            if inertia.get(key) is not None:
                inertia.set(key, repr(args.scale * float(inertia.get(key))))

    if any(offset):
        origin = inertial.find("origin")
        if origin is None:
            origin = ET.SubElement(inertial, "origin")
        xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
        origin.set("xyz", " ".join(repr(v + d) for v, d in zip(xyz, offset)))

tree.write(args.out)
print(f"wrote {args.out}: {touched} links, inertia x{args.scale:g}, mass "
      f"x{args.mass_scale:g}, com offset {offset} m, selected mass "
      f"{selected_mass:.4f} kg")
