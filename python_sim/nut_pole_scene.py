import xml.etree.ElementTree as ET
import numpy as np

def add_nut_assembly_scene(model_path, output_path):
    tree = ET.parse(model_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    asset = root.find("asset")

    # Add textures
    tex1 = ET.SubElement(asset, "texture")
    tex1.set("file", "assets/textures/brass-ambra.png")
    tex1.set("type", "cube")
    tex1.set("name", "brass-metal")

    tex2 = ET.SubElement(asset, "texture")
    tex2.set("file", "assets/textures/steel-scratched.png")
    tex2.set("type", "cube")
    tex2.set("name", "steel-metal")

    mat1 = ET.SubElement(asset, "material")
    mat1.set("name", "bmetal")
    mat1.set("reflectance", "1.0")
    mat1.set("shininess", "1.0")
    mat1.set("specular", "1.0")
    mat1.set("texrepeat", "1 1")
    mat1.set("texture", "brass-metal")
    mat1.set("texuniform", "true")

    mat2 = ET.SubElement(asset, "material")
    mat2.set("name", "smetal")
    mat2.set("reflectance", "1.0")
    mat2.set("shininess", "1.0")
    mat2.set("specular", "1.0")
    mat2.set("texrepeat", "1 1")
    mat2.set("texture", "steel-metal")
    mat2.set("texuniform", "true")

    # ── Table ─────────────────────────────────────────────────
    table_body = ET.SubElement(worldbody, "body")
    table_body.set("name", "nut_table")
    table_body.set("pos", "0.45 -0.1 0.35")

    table_geom = ET.SubElement(table_body, "geom")
    table_geom.set("name", "nut_table_surface")
    table_geom.set("type", "box")
    table_geom.set("size", "0.3 0.3 0.02")
    table_geom.set("rgba", "0.8 0.8 0.8 1")
    table_geom.set("contype", "1")
    table_geom.set("conaffinity", "1")
    table_geom.set("condim", "4")
    table_geom.set("solimp", "0.99 0.999 0.001")
    table_geom.set("solref", "0.01 1")
    table_geom.set("friction", "1 0.5 0.1")

    for i, (lx, ly) in enumerate([(0.25, 0.25), (-0.25, 0.25), (0.25, -0.25), (-0.25, -0.25)]):
        leg = ET.SubElement(table_body, "geom")
        leg.set("name", f"nut_table_leg_{i}")
        leg.set("type", "cylinder")
        leg.set("size", "0.02 0.15")
        leg.set("pos", f"{lx} {ly} -0.17")
        leg.set("rgba", "0.5 0.3 0.1 1")

    # ── Square peg (fixed on table) ───────────────────────────
    peg1_body = ET.SubElement(worldbody, "body")
    peg1_body.set("name", "peg1")
    peg1_body.set("pos", "0.45 -0.1 0.38")

    peg1_col = ET.SubElement(peg1_body, "geom")
    peg1_col.set("type", "box")
    peg1_col.set("size", "0.016 0.016 0.1")
    peg1_col.set("friction", "1 0.005 0.0001")
    peg1_col.set("solimp", "0.99 0.999 0.001")
    peg1_col.set("solref", "0.01 1")
    peg1_col.set("condim", "4")
    peg1_col.set("conaffinity", "1")
    peg1_col.set("contype", "1")

    peg1_vis = ET.SubElement(peg1_body, "geom")
    peg1_vis.set("type", "box")
    peg1_vis.set("size", "0.016 0.016 0.1")
    peg1_vis.set("conaffinity", "0")
    peg1_vis.set("contype", "0")
    peg1_vis.set("material", "bmetal")

    # ── Square nut (free to move) ─────────────────────────────
    nut_body = ET.SubElement(worldbody, "body")
    nut_body.set("name", "nut")
    nut_body.set("pos", "0.35 0.1 0.4")
    nut_body.set("quat", "0 0 0 1")

    nut_joint = ET.SubElement(nut_body, "joint")
    nut_joint.set("name", "nut_free")
    nut_joint.set("type", "free")

    nut_geoms = [
        ("-0.03325 0 0",   "0.0105 0.04375 0.015"),
        ("0.0 0.03325 0",  "0.03125 0.0105 0.015"),
        ("0.0 -0.03325 0", "0.03125 0.0105 0.015"),
        ("0.03325 0 0",    "0.0105 0.04375 0.015"),
        ("0.08 0 0",      "0.05 0.015875 0.015"),
    ]
    for pos, size in nut_geoms:
        g = ET.SubElement(nut_body, "geom")
        g.set("type", "box")
        g.set("pos", pos)
        g.set("size", size)
        g.set("material", "bmetal")
        g.set("density", "1")
        g.set("friction", "2.0 1.0 0.5")
        g.set("condim", "4")
        g.set("conaffinity", "1")
        g.set("contype", "1")
        g.set("solimp", "0.99 0.999 0.001")
        g.set("solref", "0.01 1")

    tree.write(output_path)
    print(f"Nut assembly scene written to {output_path}")

if __name__ == "__main__":
    # Copy steel texture first
    import shutil, os
    os.makedirs("../models/assets/textures", exist_ok=True)
    shutil.copy(
        r"C:\Users\lma343\robosuite\robosuite\models\assets\textures\steel-scratched.png",
        "../models/assets/textures/steel-scratched.png"
    )
    add_nut_assembly_scene(
        "../models/panda.xml",
        "../models/panda_nut.xml"
    )
    print("Done!")