import numpy as np
import xml.etree.ElementTree as ET

def generate_wipe_markers(
    num_markers=10,
    table_size=(0.4, 0.4),  # half sizes
    coverage_factor=0.9,
    line_width=0.02,
    seed=None
):
    rng = np.random.default_rng(seed)
    markers = []
   
    def sample_start():
        direction = rng.uniform(-np.pi, np.pi)
        x = rng.uniform(
            -table_size[0] * coverage_factor + line_width/2,
             table_size[0] * coverage_factor - line_width/2
        )
        y = rng.uniform(
            -table_size[1] * coverage_factor + line_width/2,
             table_size[1] * coverage_factor - line_width/2
        )
        return np.array([x, y]), direction

    def sample_next(pos, direction):
        if rng.uniform(0, 1) > 0.7:
            direction += rng.normal(0, 0.5)
       
        for _ in range(100):
            x = pos[0] + 0.005 * np.sin(direction)
            y = pos[1] + 0.005 * np.cos(direction)
            if (abs(x) < table_size[0] * coverage_factor - line_width/2 and
                abs(y) < table_size[1] * coverage_factor - line_width/2):
                return np.array([x, y]), direction
            direction += rng.normal(0, 0.5)
        return pos, direction

    pos, direction = sample_start()
    for i in range(num_markers):
        markers.append((f"marker_{i}", pos.copy()))
        pos, direction = sample_next(pos, direction)
   
    return markers

def add_wipe_scene_to_model(model_path, output_path, num_markers=20):
    tree = ET.parse(model_path)
    root = tree.getroot()
   
    # Find worldbody
    worldbody = root.find("worldbody")
   
    # Table dimensions
    table_x = 0.4  # half size
    table_y = 0.4  # half size
    table_z = 0.02  # half height
    table_height = 0.4  # height of table surface from ground
   
    # Add table
    table_body = ET.SubElement(worldbody, "body")
    table_body.set("name", "wipe_table")
    table_body.set("pos", f"0.5 0 {table_height}")
   
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
    markers = generate_wipe_markers(
        num_markers=num_markers,
        table_size=(table_x, table_y)
    )
   
    for name, (mx, my) in markers:
        marker_body = ET.SubElement(table_body, "body")
        marker_body.set("name", name)
        marker_body.set("pos", f"{mx} {my} {table_z + 0.001}")
       
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
        num_markers=20
    )
    print(f"Markers: {markers}")