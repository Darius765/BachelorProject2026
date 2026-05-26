import xml.etree.ElementTree as ET

def add_drawer_scene(model_path, output_path):
    tree = ET.parse(model_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    asset = root.find("asset")

    defaults = root.find("default")
    if defaults is None:
        defaults = ET.Element("default")
        asset_idx = list(root).index(asset)
        root.insert(asset_idx + 1, defaults)

    # Add slidecabinet default class
    slide_default = ET.SubElement(defaults, "default")
    slide_default.set("class", "slidecabinet")

    slide_joint_default = ET.SubElement(slide_default, "joint")
    slide_joint_default.set("damping", "2")
    slide_joint_default.set("frictionloss", "2")
    slide_joint_default.set("armature", ".01")
    slide_joint_default.set("limited", "true")

    slide_geom_default = ET.SubElement(slide_default, "geom")
    slide_geom_default.set("conaffinity", "0")
    slide_geom_default.set("contype", "0")
    slide_geom_default.set("group", "1")
    slide_geom_default.set("material", "M_slide_blue")
    slide_geom_default.set("type", "mesh")

    slide_col_default = ET.SubElement(slide_default, "default")
    slide_col_default.set("class", "slide_collision")

    slide_col_geom = ET.SubElement(slide_col_default, "geom")
    slide_col_geom.set("conaffinity", "1")
    slide_col_geom.set("condim", "3")
    slide_col_geom.set("contype", "0")
    slide_col_geom.set("group", "4")
    slide_col_geom.set("margin", "0.001")
    slide_col_geom.set("material", "slide_collision_blue")

    # Add slidecabinet assets
    tex1 = ET.SubElement(asset, "texture")
    tex1.set("name", "T_slide_metal")
    tex1.set("type", "cube")
    tex1.set("height", "1")
    tex1.set("width", "1")
    tex1.set("file", "furniture_sim/common/textures/metal0.png")

    tex2 = ET.SubElement(asset, "texture")
    tex2.set("name", "T_slide_wood")
    tex2.set("type", "cube")
    tex2.set("height", "1")
    tex2.set("width", "1")
    tex2.set("file", "furniture_sim/common/textures/wood1.png")

    mat1 = ET.SubElement(asset, "material")
    mat1.set("name", "M_slide_metal")
    mat1.set("texture", "T_slide_metal")
    mat1.set("texrepeat", "3 3")
    mat1.set("reflectance", "0.7")
    mat1.set("shininess", ".4")
    mat1.set("texuniform", "false")

    mat2 = ET.SubElement(asset, "material")
    mat2.set("name", "M_slide_blue")
    mat2.set("texture", "T_slide_wood")
    mat2.set("rgba", "1 1 1 1")
    mat2.set("reflectance", "0.7")
    mat2.set("shininess", ".4")
    mat2.set("texuniform", "false")

    mat3 = ET.SubElement(asset, "material")
    mat3.set("name", "slide_collision_blue")
    mat3.set("rgba", "0.3 0.3 1.0 0.5")
    mat3.set("shininess", "0")
    mat3.set("specular", "0")

    mat4 = ET.SubElement(asset, "material")
    mat4.set("name", "plate-mat")
    mat4.set("rgba", "0.9 0.9 0.9 1")
    mat4.set("specular", "0.5")
    mat4.set("shininess", "0.3")

    # ── Table ─────────────────────────────────────────────────
    table_body = ET.SubElement(worldbody, "body")
    table_body.set("name", "drawer_table")
    table_body.set("pos", "0.45 -0.1 0.35")

    table_geom = ET.SubElement(table_body, "geom")
    table_geom.set("name", "drawer_table_surface")
    table_geom.set("type", "box")
    table_geom.set("size", "0.3 0.3 0.02")
    table_geom.set("rgba", "0.8 0.8 0.8 1")

    for i, (lx, ly) in enumerate([(0.25, 0.25), (-0.25, 0.25), (0.25, -0.25), (-0.25, -0.25)]):
        leg = ET.SubElement(table_body, "geom")
        leg.set("name", f"drawer_table_leg_{i}")
        leg.set("type", "cylinder")
        leg.set("size", "0.02 0.15")
        leg.set("pos", f"{lx} {ly} -0.17")
        leg.set("rgba", "0.5 0.3 0.1 1")

    # ── Slide cabinet on table ────────────────────────────────
    cabinet_body = ET.SubElement(worldbody, "body")
    cabinet_body.set("name", "slide")
    cabinet_body.set("pos", "0.55 -0.1 0.62")

    # Cabinet body geoms (from slidecabinet_body.xml)
    cab_geoms = [
        ("-0.225 0 -0.18", "0.223 0.3 0.02", "box", "M_slide_blue"),
        ("0.224 0 0",      "0.226 0.3 0.2",  "box", "M_slide_blue"),
        ("-0.225 0 0.18",  "0.223 0.3 0.02", "box", "M_slide_blue"),
        ("-0.426 0 0",     "0.022 0.3 0.16", "box", "M_slide_blue"),
        ("-0.2 0.276 0.0", "0.21 0.024 0.16","box", "M_slide_blue"),
    ]
    for pos, size, gtype, mat in cab_geoms:
        g = ET.SubElement(cabinet_body, "geom")
        g.set("pos", pos)
        g.set("size", size)
        g.set("type", gtype)
        g.set("material", mat)
        g.set("conaffinity", "0")
        g.set("contype", "0")
        g.set("group", "1")

    # Collision geoms for cabinet
    for pos, size, gtype in [
        ("-0.225 0 -0.18", "0.223 0.3 0.02", "box"),
        ("0.224 0 0",      "0.226 0.3 0.2",  "box"),
        ("-0.225 0 0.18",  "0.223 0.3 0.02", "box"),
        ("-0.426 0 0",     "0.022 0.3 0.16", "box"),
        ("-0.2 0.276 0",   "0.2 0.024 0.16", "box"),
    ]:
        g = ET.SubElement(cabinet_body, "geom")
        g.set("pos", pos)
        g.set("size", size)
        g.set("type", gtype)
        g.set("conaffinity", "1")
        g.set("condim", "3")
        g.set("contype", "0")
        g.set("group", "4")
        g.set("material", "slide_collision_blue")

    # Sliding door body
    slidelink = ET.SubElement(cabinet_body, "body")
    slidelink.set("name", "slidelink")
    slidelink.set("pos", "-0.225 -0.32 0")

    slide_joint = ET.SubElement(slidelink, "joint")
    slide_joint.set("name", "slidedoor_joint")
    slide_joint.set("axis", "1 0 0")
    slide_joint.set("type", "slide")
    slide_joint.set("range", "0 .44")
    slide_joint.set("damping", "2")
    slide_joint.set("frictionloss", "2")

    # Handle geoms
    for euler, pos, size, gtype in [
        ("1.57 0 0", "-0.183 -0.06 -0.114", "0.019 0.053 0.019", "cylinder"),
        ("1.57 0 0", "-0.183 -0.06 0.114",  "0.019 0.053 0.019", "cylinder"),
        ("0 0 0",    "-0.183 -0.123 0",      "0.022 0.159",       "cylinder"),
    ]:
        g = ET.SubElement(slidelink, "geom")
        g.set("material", "M_slide_metal")
        if euler != "0 0 0":
            g.set("euler", euler)
        g.set("pos", pos)
        g.set("size", size)
        g.set("type", gtype)

    # Door panel
    g = ET.SubElement(slidelink, "geom")
    g.set("pos", "0 -.02 0")
    g.set("size", "0.225 0.03 0.195")
    g.set("type", "box")
    g.set("material", "M_slide_blue")
    g.set("conaffinity", "1")
    g.set("condim", "3")
    g.set("contype", "0")
    g.set("group", "4")

    # ── Plate ─────────────────────────────────────────────────
    plate_body = ET.SubElement(worldbody, "body")
    plate_body.set("name", "plate")
    plate_body.set("pos", "0.25 -0.1 0.40")

    plate_joint = ET.SubElement(plate_body, "joint")
    plate_joint.set("name", "plate_free")
    plate_joint.set("type", "free")

    plate_geom = ET.SubElement(plate_body, "geom")
    plate_geom.set("name", "plate_geom")
    plate_geom.set("type", "cylinder")
    plate_geom.set("size", "0.06 0.005")
    plate_geom.set("material", "plate-mat")
    plate_geom.set("density", "200")
    plate_geom.set("friction", "0.8 0.3 0.1")

    tree.write(output_path)
    print(f"Drawer scene written to {output_path}")

if __name__ == "__main__":
    add_drawer_scene(
        "../models/panda.xml",
        "../models/panda_drawer.xml"
    )
    print("Done!")