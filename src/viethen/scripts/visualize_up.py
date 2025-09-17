import vtk
import numpy as np
import sys

def load_stl(file_path):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(file_path)
    reader.Update()
    return reader.GetOutput()

def color_up_facing_polygons(polydata, up_vector, threshold_angle=20):
    threshold_radians = np.radians(threshold_angle)
    normal_generator = vtk.vtkPolyDataNormals()
    normal_generator.SetInputData(polydata)
    normal_generator.ComputeCellNormalsOn()
    normal_generator.Update()
    normals = normal_generator.GetOutput().GetCellData().GetNormals()
    cell_colors = vtk.vtkUnsignedCharArray()
    cell_colors.SetNumberOfComponents(3)
    cell_colors.SetName("Colors")
    up_vector = np.array(up_vector)

    for i in range(polydata.GetNumberOfCells()):
        normal = np.array(normals.GetTuple(i))
        cos_angle = np.dot(normal, up_vector) / (np.linalg.norm(normal) * np.linalg.norm(up_vector))
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        if angle <= threshold_radians:
            cell_colors.InsertNextTuple3(255, 0, 0)
        else:
            cell_colors.InsertNextTuple3(255, 255, 255)

    polydata.GetCellData().SetScalars(cell_colors)
    return polydata

def create_slider_widget(interactor, callback_func, min_value=0.0, max_value=90.0, initial_value=20.0):
    slider_rep = vtk.vtkSliderRepresentation2D()
    slider_rep.SetMinimumValue(min_value)
    slider_rep.SetMaximumValue(max_value)
    slider_rep.SetValue(initial_value)
    slider_rep.SetTitleText("Angle Threshold")
    slider_rep.GetPoint1Coordinate().SetCoordinateSystemToNormalizedDisplay()
    slider_rep.GetPoint1Coordinate().SetValue(0.1, 0.1)
    slider_rep.GetPoint2Coordinate().SetCoordinateSystemToNormalizedDisplay()
    slider_rep.GetPoint2Coordinate().SetValue(0.4, 0.1)
    slider_rep.SetSliderLength(0.02)
    slider_rep.SetSliderWidth(0.03)
    slider_rep.SetEndCapLength(0.01)
    slider_rep.SetEndCapWidth(0.03)
    slider_rep.SetTubeWidth(0.005)
    slider_rep.SetLabelFormat("%3.0lf")
    slider_rep.SetTitleHeight(0.02)
    slider_rep.SetLabelHeight(0.02)
    
    slider_widget = vtk.vtkSliderWidget()
    slider_widget.SetInteractor(interactor)
    slider_widget.SetRepresentation(slider_rep)
    slider_widget.KeyPressActivationOff()
    slider_widget.SetAnimationModeToAnimate()
    slider_widget.SetEnabled(True)
    slider_widget.AddObserver("EndInteractionEvent", callback_func)
    return slider_widget

def add_text_overlay(renderer, text):
    text_actor = vtk.vtkTextActor()
    text_actor.SetInput(text)
    text_actor.GetTextProperty().SetFontSize(24)
    text_actor.GetTextProperty().SetColor(1.0, 1.0, 1.0)
    text_actor.SetPosition(20, 30)
    renderer.AddActor(text_actor)

def main():
    if len(sys.argv) < 2:
        print("Usage: python color_up_facing_polygons.py <path_to_stl_file>")
        sys.exit(1)

    file_path = sys.argv[1]
    threshold_angle = 20
    up_vector = np.array([0, 0, 1])

    polydata = load_stl(file_path)
    colored_polydata = color_up_facing_polygons(polydata, up_vector, threshold_angle)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(colored_polydata)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    renderer = vtk.vtkRenderer()
    render_window = vtk.vtkRenderWindow()
    render_window_interactor = vtk.vtkRenderWindowInteractor()

    renderer.AddActor(actor)
    renderer.SetBackground(0.1, 0.1, 0.1)

    render_window.AddRenderer(renderer)
    render_window.SetSize(800, 800)
    render_window_interactor.SetRenderWindow(render_window)

    def keypress_callback(obj, event):
        key = obj.GetKeySym()
        if key == 's':
            nonlocal up_vector
            up_vector = -up_vector  # Flip the up vector
            colored_polydata = color_up_facing_polygons(polydata, up_vector, threshold_angle)
            mapper.SetInputData(colored_polydata)
            render_window.Render()

    def slider_callback(obj, event):
        slider_value = obj.GetRepresentation().GetValue()
        colored_polydata = color_up_facing_polygons(polydata, up_vector, slider_value)
        mapper.SetInputData(colored_polydata)
        render_window.Render()

    slider_widget = create_slider_widget(render_window_interactor, slider_callback)
    render_window_interactor.AddObserver('KeyPressEvent', keypress_callback)
    add_text_overlay(renderer, "Press 's' to flip direction vector")

    render_window.Render()
    render_window_interactor.Start()

if __name__ == "__main__":
    main()
