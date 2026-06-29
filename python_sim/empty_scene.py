import xml.etree.ElementTree as ET

def add_empty_scene(model_path, output_path):
    tree = ET.parse(model_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    asset = root.find("asset")

    # Add wood texture
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

    # ── Table ─────────────────────────────────────────────────
    table_body = ET.SubElement(worldbody, "body")
    table_body.set("name", "empty_table")
    table_body.set("pos", "0.5 -0.1 0.4")

    table_geom = ET.SubElement(table_body, "geom")
    table_geom.set("name", "empty_table_surface")
    table_geom.set("type", "box")
    table_geom.set("size", "0.35 0.3 0.02")
    table_geom.set("material", "wood-mat")
    table_geom.set("contype", "1")
    table_geom.set("conaffinity", "1")
    table_geom.set("friction", "1 0.5 0.1")
    table_geom.set("solimp", "0.99 0.999 0.001")
    table_geom.set("solref", "0.01 1")
    table_geom.set("condim", "4")

    for i, (lx, ly) in enumerate([(0.3, 0.25), (-0.25, 0.25), (0.3, -0.25), (-0.25, -0.25)]):
        leg = ET.SubElement(table_body, "geom")
        leg.set("name", f"empty_table_leg_{i}")
        leg.set("type", "cylinder")
        leg.set("size", "0.02 0.2")
        leg.set("pos", f"{lx} {ly} -0.22")
        leg.set("material", "wood-mat")

    # ── Small cube ────────────────────────────────────────────
    cube_body = ET.SubElement(worldbody, "body")
    cube_body.set("name", "cube")
    cube_body.set("pos", "0.45 0.0 0.43")

    cube_joint = ET.SubElement(cube_body, "joint")
    cube_joint.set("name", "cube_free")
    cube_joint.set("type", "free")

    cube_geom = ET.SubElement(cube_body, "geom")
    cube_geom.set("name", "cube_geom")
    cube_geom.set("type", "box")
    cube_geom.set("size", "0.03 0.03 0.03")
    cube_geom.set("rgba", "0.2 0.5 0.8 1")
    cube_geom.set("density", "200")
    cube_geom.set("friction", "2.0 1.0 0.5")
    cube_geom.set("contype", "1")
    cube_geom.set("conaffinity", "1")
    cube_geom.set("solimp", "0.99 0.999 0.001")
    cube_geom.set("solref", "0.02 1")
    cube_geom.set("condim", "4")

    # ── Small cube 2 ────────────────────────────────────────────
    cube2_body = ET.SubElement(worldbody, "body")
    cube2_body.set("name", "cube2")
    cube2_body.set("pos", "0.45 -0.2 0.43")

    cube2_joint = ET.SubElement(cube2_body, "joint")
    cube2_joint.set("name", "cube2_free")
    cube2_joint.set("type", "free")

    cube2_geom = ET.SubElement(cube2_body, "geom")
    cube2_geom.set("name", "cube2_geom")
    cube2_geom.set("type", "box")
    cube2_geom.set("size", "0.03 0.03 0.03")
    cube2_geom.set("rgba", "0.2 0.5 0.8 1")
    cube2_geom.set("density", "200")
    cube2_geom.set("friction", "2.0 1.0 0.5")
    cube2_geom.set("contype", "1")
    cube2_geom.set("conaffinity", "1")
    cube2_geom.set("solimp", "0.99 0.999 0.001")
    cube2_geom.set("solref", "0.02 1")
    cube2_geom.set("condim", "4")
    

    tree.write(output_path)
    print(f"Empty scene written to {output_path}")

if __name__ == "__main__":
    add_empty_scene(
        "../models/panda.xml",
        "../models/panda_empty.xml"
    )
    print("Done!")