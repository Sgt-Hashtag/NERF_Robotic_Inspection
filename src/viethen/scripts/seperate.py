import vtk
import argparse
import os

def separate_meshes(input_file, output_directory, min_size):
    # Read the STL file
    reader = vtk.vtkSTLReader()
    reader.SetFileName(input_file)
    reader.Update()

    # Get the polydata (mesh) from the reader
    polydata = reader.GetOutput()

    # Filter to find connected components
    connectivity_filter = vtk.vtkPolyDataConnectivityFilter()
    connectivity_filter.SetInputData(polydata)
    connectivity_filter.SetExtractionModeToAllRegions()
    connectivity_filter.ColorRegionsOn()  # Useful to visualize the different components
    connectivity_filter.Update()

    num_regions = connectivity_filter.GetNumberOfExtractedRegions()
    print(f"Found {num_regions} continuous meshes in the STL file.")

    valid_meshes = 0
    for i in range(num_regions):
        # Extract each region (continuous mesh)
        region_extractor = vtk.vtkPolyDataConnectivityFilter()
        region_extractor.SetInputData(polydata)
        region_extractor.SetExtractionModeToSpecifiedRegions()
        region_extractor.AddSpecifiedRegion(i)
        region_extractor.Update()

        # Get the mesh for this region
        region_mesh = region_extractor.GetOutput()

        # Check the size of the region (number of cells)
        if region_mesh.GetNumberOfCells() >= min_size:
            # Write the extracted region to a new STL file if it meets the size requirement
            output_filename = os.path.join(output_directory, f"mesh_part_{valid_meshes}.stl")
            writer = vtk.vtkSTLWriter()
            writer.SetFileTypeToBinary() # rviz only supports Binary
            writer.SetFileName(output_filename)
            writer.SetInputData(region_mesh)
            writer.Write()

            print(f"Mesh part {valid_meshes} written to {output_filename}")
            valid_meshes += 1
        else:
            print(f"Skipped mesh part {i} (size {region_mesh.GetNumberOfCells()} cells)")

    print(f"Saved {valid_meshes} valid mesh parts.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Separate an STL file into continuous meshes.")
    parser.add_argument('-i', '--input', type=str, required=True, help="Path to the input STL file.")
    parser.add_argument('-o', '--output', type=str, required=True, help="Directory to save the output mesh files.")
    parser.add_argument('-m', '--min_size', type=int, default=100, help="Minimum size (number of cells) for a mesh to be saved.")

    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: The input file {args.input} does not exist.")
        exit(1)

    if not os.path.exists(args.output):
        os.makedirs(args.output)
        print(f"Created output directory: {args.output}")

    separate_meshes(args.input, args.output, args.min_size)
