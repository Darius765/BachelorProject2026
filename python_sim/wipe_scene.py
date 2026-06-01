import numpy as np
import xml.etree.ElementTree as ET

def generate_wipe_markers(num_markers=10):
    
    fixed_positions = [
        (-0.23, 0.06),
        (-0.18, -0.12),
        (-0.08, 0.08),
        (-0.02, 0.01),
        (-0.10, -0.20),
        (-0.17, 0.10),
        (-0.30, -0.10),
        (-0.13, 0.15),
        (-0.20, -0.05),
        (-0.22, -0.20),
    ]

    markers = []
    for i, (x,y) in enumerate(fixed_positions[:num_markers]):
        markers.append((f"marker_{i}", np.array([x, y])))
    return markers

def add_wipe_scene_to_model(model_path, output_path, num_markers=20):
    tree = ET.parse(model_path)
    root = tree.getroot()
   
    # Find worldbody
    worldbody = root.find("worldbody")
   
    # Table dimensions
    table_x = 0.4
    table_y = 0.4
    table_z = 0.02
    table_height = 0.40
   
    # Add table
    table_body = ET.SubElement(worldbody, "body")
    table_body.set("name", "wipe_table")
    table_body.set("pos", f"0.55 -0.15 {table_height}")
   
    table_geom = ET.SubElement(table_body, "geom")
    table_geom.set("name", "wipe_table_surface")
    table_geom.set("type", "box")
    table_geom.set("size", f"{table_x} {table_y} {table_z}")
    table_geom.set("rgba", "0.8 0.8 0.8 1")
    table_geom.set("friction", "0.01 0.005 0.0001")
   
    # Add table legs
    for i, (lx, ly) in enumerate([(0.35, 0.35), (-0.35, 0.35), (0.35, -0.35), (-0.35, -0.35)]):
        leg = ET.SubElement(table_body, "geom")
        leg.set("name", f"table_leg_{i}")
        leg.set("type", "cylinder")
        leg.set("size", "0.02 0.2")
        leg.set("pos", f"{lx} {ly} -0.22")
        leg.set("rgba", "0.5 0.3 0.1 1")

    # Generate and add wipe markers
    markers = generate_wipe_markers(num_markers=num_markers)
   
    for name, (mx, my) in markers:
        marker_body = ET.SubElement(table_body, "body")
        marker_body.set("name", name)
        marker_body.set("pos", f"{mx} {my + 0.1} {table_z + 0.001}")
       
        marker_geom = ET.SubElement(marker_body, "geom")
        marker_geom.set("name", f"{name}_geom")
        marker_geom.set("type", "cylinder")
        marker_geom.set("size", "0.01 0.001")
        marker_geom.set("rgba", "0.6 0.3 0.1 1")
        marker_geom.set("contype", "0")
        marker_geom.set("conaffinity", "0")
   
    # Write output
    tree.write(output_path)
    print(f"Wipe scene written to {output_path}")
    print(f"Table at height {table_height}, {num_markers} markers placed")
    return [name for name, _ in markers]

if __name__ == "__main__":
    markers = add_wipe_scene_to_model(
        "../models/panda.xml",
        "../models/panda_wipe.xml",
        num_markers=10
    )
    print(f"Markers: {markers}")