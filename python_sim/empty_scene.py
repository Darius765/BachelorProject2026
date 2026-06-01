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

    for i, (lx, ly) in enumerate([(0.3, 0.25), (-0.25, 0.25), (0.3, -0.25), (-0.25, -0.25)]):
        leg = ET.SubElement(table_body, "geom")
        leg.set("name", f"empty_table_leg_{i}")
        leg.set("type", "cylinder")
        leg.set("size", "0.02 0.2")
        leg.set("pos", f"{lx} {ly} -0.22")
        leg.set("material", "wood-mat")

    tree.write(output_path)
    print(f"Empty scene written to {output_path}")

if __name__ == "__main__":
    add_empty_scene(
        "../models/panda.xml",
        "../models/panda_empty.xml"
    )
    print("Done!")