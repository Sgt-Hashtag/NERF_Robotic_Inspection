import trimesh

def convert_obj_to_stl(input_file, output_file):
    mesh = trimesh.load(input_file, file_type='obj')
    mesh.export(output_file, file_type='stl')

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Convert OBJ to STL")
    parser.add_argument('input', type=str, help="Input OBJ file path")
    parser.add_argument('output', type=str, help="Output STL file path")

    args = parser.parse_args()

    convert_obj_to_stl(args.input, args.output)
