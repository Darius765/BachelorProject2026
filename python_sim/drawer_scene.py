import xml.etree.ElementTree as ET
import os

def add_drawer_scene(model_path, output_path):
    tree = ET.parse(model_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    asset = root.find("asset")

    # Add textures
    tex = ET.SubElement(asset, "texture")
    tex.set("builtin", "flat")
    tex.set("height", "512")
    tex.set("width", "512")
    tex.set("rgb1", "0.6 0.4 0.2")
    tex.set("rgb2", "0.6 0.4 0.2")
    tex.set("name", "wood-tex")
    tex.set("type", "cube")

    mat = ET.SubElement(asset, "material")
    mat.set("name", "wood-mat")
    mat.set("texture", "wood-tex")
    mat.set("specular", "0.2")
    mat.set("shininess", "0.1")

    mat2 = ET.SubElement(asset, "material")
    mat2.set("name", "metal-mat")
    mat2.set("rgba", "0.7 0.7 0.7 1")
    mat2.set("specular", "0.8")
    mat2.set("shininess", "0.5")

    mat3 = ET.SubElement(asset, "material")
    mat3.set("name", "bowl-mat")
    mat3.set("rgba", "0.9 0.9 0.9 1")
    mat3.set("specular", "0.5")
    mat3.set("shininess", "0.3")

    # ── Table ─────────────────────────────────────────────────
    table_body = ET.SubElement(worldbody, "body")
    table_body.set("name", "drawer_table")
    table_body.set("pos", "0.45 -0.1 0.35")

    table_geom = ET.SubElement(table_body, "geom")
    table_geom.set("name", "drawer_table_surface")
    table_geom.set("type", "box")
    table_geom.set("size", "0.3 0.3 0.02")
    table_geom.set("material", "wood-mat")

    for i, (lx, ly) in enumerate([(0.35, 0.25), (-0.05, 0.25), (0.35, -0.25), (-0.05, -0.25)]):
        leg = ET.SubElement(table_body, "geom")
        leg.set("name", f"drawer_table_leg_{i}")
        leg.set("type", "cylinder")
        leg.set("size", "0.02 0.15")
        leg.set("pos", f"{lx} {ly} -0.17")
        leg.set("material", "wood-mat")

    # ── Cabinet + drawer (drawer is child of cabinet) ──────────
    cabinet_body = ET.SubElement(worldbody, "body")
    cabinet_body.set("name", "cabinet")
    cabinet_body.set("pos", "0.70 -0.1 0.45")
    cabinet_body.set("euler", "0 0 3.14159")

    # Cabinet walls
    for name, pos, size in [
        ("cab_top",    "0 0  0.08",  "0.15 0.12 0.01"),
        ("cab_bottom", "0 0 -0.08",  "0.15 0.12 0.01"),
        ("cab_left",   "0  0.11 0",  "0.15 0.01 0.08"),
        ("cab_right",  "0 -0.11 0",  "0.15 0.01 0.08"),
        ("cab_back",   "-0.14 0 0",  "0.01 0.12 0.08"),
    ]:
        g = ET.SubElement(cabinet_body, "geom")
        g.set("name", name)
        g.set("type", "box")
        g.set("pos", pos)
        g.set("size", size)
        g.set("material", "wood-mat")

    # ── Drawer (child of cabinet) ──────────────────────────────
    drawer_body = ET.SubElement(cabinet_body, "body")
    drawer_body.set("name", "drawer")
    drawer_body.set("pos", "0 0 0")

    drawer_joint = ET.SubElement(drawer_body, "joint")
    drawer_joint.set("name", "drawer_slide")
    drawer_joint.set("type", "slide")
    drawer_joint.set("axis", "1 0 0")
    drawer_joint.set("range", "0 0.2")
    drawer_joint.set("damping", "5")
    drawer_joint.set("frictionloss", "2")

    # Drawer bottom
    g = ET.SubElement(drawer_body, "geom")
    g.set("name", "drawer_bottom")
    g.set("type", "box")
    g.set("size", "0.13 0.10 0.005")
    g.set("pos", "0 0 -0.065")
    g.set("material", "wood-mat")

    # Drawer walls
    for name, pos, size in [
        ("drawer_left",  "0  0.09 0",  "0.13 0.005 0.06"),
        ("drawer_right", "0 -0.09 0",  "0.13 0.005 0.06"),
        ("drawer_back",  "-0.12 0 0",  "0.005 0.10 0.06"),
    ]:
        g = ET.SubElement(drawer_body, "geom")
        g.set("name", name)
        g.set("type", "box")
        g.set("pos", pos)
        g.set("size", size)
        g.set("material", "wood-mat")

    # Drawer front panel
    g = ET.SubElement(drawer_body, "geom")
    g.set("name", "drawer_front")
    g.set("type", "box")
    g.set("size", "0.005 0.12 0.075")
    g.set("pos", "0.135 0 0")
    g.set("material", "wood-mat")

    # Drawer handle
    g = ET.SubElement(drawer_body, "geom")
    g.set("name", "drawer_handle")
    g.set("type", "cylinder")
    g.set("size", "0.008 0.04")
    g.set("pos", "0.145 0 0.04")
    g.set("quat", "0.7071 0 0.7071 0")
    g.set("material", "metal-mat")

    
    # ── Bowl ──────────────────────────────────────────────────
    bowl_body = ET.SubElement(worldbody, "body")
    bowl_body.set("name", "bowl")
    bowl_body.set("pos", "0.25 -0.1 0.40")

    bowl_joint = ET.SubElement(bowl_body, "joint")
    bowl_joint.set("name", "bowl_free")
    bowl_joint.set("type", "free")

    # Bowl base
    g = ET.SubElement(bowl_body, "geom")
    g.set("name", "bowl_base")
    g.set("type", "cylinder")
    g.set("size", "0.055 0.005")
    g.set("pos", "0 0 -0.02")
    g.set("material", "bowl-mat")
    g.set("density", "200")
    g.set("friction", "0.8 0.3 0.1")

    # Bowl walls — 4 thin boxes arranged in a ring
    for name, pos, size in [
        ("bowl_wall_front", "0  0.05 0.01",  "0.055 0.005 0.03"),
        ("bowl_wall_back",  "0 -0.05 0.01",  "0.055 0.005 0.03"),
        ("bowl_wall_left",  "0.05 0 0.01",   "0.005 0.055 0.03"),
        ("bowl_wall_right", "-0.05 0 0.01",  "0.005 0.055 0.03"),
    ]:
        g = ET.SubElement(bowl_body, "geom")
        g.set("name", name)
        g.set("type", "box")
        g.set("pos", pos)
        g.set("size", size)
        g.set("material", "bowl-mat")
        g.set("density", "100")
        g.set("friction", "0.8 0.3 0.1")

    tree.write(output_path)
    print(f"Drawer scene written to {output_path}")

if __name__ == "__main__":
    add_drawer_scene(
        "../models/panda.xml",
        "../models/panda_drawer.xml"
    )
    print("Done!")